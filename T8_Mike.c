/**
 * @file RWD_Team8.c
 * C Program to Read in I.Bus or S.Bus from FlySky Receiver and control RWD Robot for Team 8 Season 2
 *
 * To Compile file, in terminal type 'make T8' and press enter
 * To run type ./T8 
 *
 *
 *
 * Team 8 Beaglebone Pin Usage:
 * 	GP0 GPIO1_17 Right Ultrasonic TRIG
 * 	GP0 GPIO1_25 Right Ultrasonic ECHO
 * 	GP0 GPIO3_17 Left Ultrasonic TRIG
 * 	GP0 GPIO3_20 Left Ultrasonic ECHO
 * 	GP1 GPIO3_1 Front Limit Switch
 * 	GP1 GPIO3_2 Rear Limit Switch
 * 	ADCS AIN0 Phototransistor
 * 	QE3 Left Wheel Encoder
 * 	QE4 Right Wheel Encoder
 *  UART0 Flysky Receiver
 * 	I2C Arm Encoder
 * 	Servo 1 Left Wheel
 * 	Servo 2 Arm
 * 	Servo 3 Front EM
 * 	Servo 5 Right Wheel
 * 	Servo 7 Rear EM
 *
 */

#include <math.h>
#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc/time.h>  				// For Time


//Added by Michael
//For ESC control
#include <getopt.h>
#include <signal.h>
#include <rc/servo.h>				//ESC Control pins '1-5', '0' will send signal to all pins
#include <rc/adc.h>
#include <rc/dsm.h>

#include <stdlib.h> 				// UART1 Flysky Receiver  for atoi
#include <string.h> 				// UART1 Flysky Receiver
#include <rc/uart.h> 				// UART1 Flysky Receiver
#include <inttypes.h> // for PRIu64

#define BUF_SIZE        32 			// UART1 Flysky Receiver
#define TIMEOUT_S       0.5 		// UART1 Flysky Receiver
#define BAUDRATE        115200 		// UART1 Flysky Receiver
#define RECEIVER		1

#define DESKTOP //used when testing robot in a desktop environment. Comment out when compiling for bot usage
#define BOT //Undecided but opposite 

 // function declarations
void on_pause_press();
void on_pause_release();
void channel_update();
void orientation();
void mode_select();
void arm_adjustment();
void operation_percent();
void drive_values();

//Added by Michael
//void print_data(uint16_t, *ibus_data, int ch);
//double interp(double point, double *table);
 
struct ibus_state {
  uint_fast8_t state;
  uint_fast16_t checksum;
  uint_fast8_t datal;
  uint_fast8_t channel_count;
};

void ibus_init(struct ibus_state *state, uint_fast8_t channel_count);
int ibus_read(struct ibus_state *state, uint16_t *data, uint8_t ch);

/**
 * At startup the program runs the main function. 
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	
	uint64_t prev50ms = 0;  	// init task timers
	uint64_t prev40ms = 0;		// init task timers
	uint64_t prev100ms = 0;		// init task timers
	uint64_t curms; 			// init task timers
	int ret = 0;
	uint8_t *_ch, chmain;
	struct ibus_state *_state, statemain;
	uint16_t *_data, datamain[14];
	//Added by Michael================================
	uint8_t rx_ch[10] = {1,2,3,4,5,7,8,9,10}; //normalized rx input
	int rx_val[2]={0,0};
	
	uint64_t prevE1ms = 0;		// init task timers
	uint64_t prevE2ms = 0;		// init task timers
	uint64_t dt = 0;
    //PID Variables=====================
	double integral = 0.0;
	double derivative = 0.0;
	double Kp = 0.0;
	double Kd = 0.0;
	double Ki = 0.0;
	double error = 0.0;
	double prev_error =0.0;
	//==================================
		
	int i=0;
	double vel[2] ={0,0};
	int wheel_enc_data[2] = {0,0};
	int prev_wheel_enc_data[2] = {0,0};
	uint8_t q_encoder_pin[2]= {4,4};
	uint8_t i2c_data = 0;
	double speed_ratio = 0.0;
	int g=0;
	//Drive Logic--------------------------------------------
	int drive = 0b0000;//Potentially using 4-bit binary to describe current control state [bit1][bit2][bit3][bit4]
	int drive_state = 0;
	int esc_ch[5] = {1,2,3,4,5}; //ESC channels: 1 - R, 2 - L, 3 - O, 4 - EMR, 5 - EML
	int esc_ctrl[5] = {0,0,0,0,0};

	
	//double drive_table[4][4]={0,50,200,500}{0,0.2,0.7,1.0};
	///

	///State------------------------------------------------
	//enum robot_state {INIT, DEBUG, RUNNING, STOP, CALIBRATE}state;
	
	////
	
	//state = INIT;
	_ch = &chmain;
	_state = &statemain;
	_data = datamain;
	
	memset(_data,0,30);

	//Added by Michael
	rc_servo_init();
	rc_servo_power_rail_en(0);

	//Setup Functions
		// make sure another instance isn't running
		// if return value is -3 then a background process is running with
		// higher privaledges and we couldn't kill it, in which case we should
		// not continue or there may be hardware conflicts. If it returned -4
		// then there was an invalid argument that needs to be fixed.
		if (rc_kill_existing_process(2.0) < -2) return -1;

		// start signal handler so we can exit cleanly
		if (rc_enable_signal_handler() == -1) {
			fprintf(stderr, "ERROR: failed to start signal handler\n");
//			return -1;
		}

		// initialize pause button
		// Likely will remove Pause button functionality, not needed in robot
		if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
			RC_BTN_DEBOUNCE_DEFAULT_US)) {
			fprintf(stderr, "ERROR: failed to initialize pause button\n");
//			return -1;
		}

		// Assign functions to be called when button events occur
		rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, on_pause_release);

		// make PID file to indicate your project is running
		// due to the check made on the call to rc_kill_existing_process() above
		// we can be fairly confident there is no PID file already and we can
		// make our own safely.
		rc_make_pid_file();
		if (rc_servo_init()) return -1; //Initialize PRU
		
		//init quad encoder
		if(rc_encoder_init()){
                fprintf(stderr,"ERROR: failed to run rc_encoder_init\n");
                return -1;
        }
		//-------------------------------------------------------
		//rc_servo_power_rail_en(1); //servo rails on DO NOT POWER RAILS in actual robot code
		rc_servo_send_esc_pulse_normalized(1,-0.1);
		printf("\nPress and release pause button to turn green LED on and off\n");
		printf("hold pause button down for 2 seconds to exit\n");

		// Keep looping until state changes to EXITING
		rc_set_state(RUNNING);
	
	
	    ibus_init(_state, 10);   // set _state to 0 to start looking for a new header and how many channels to read in
	  
				
	    if(rc_uart_init(RECEIVER, BAUDRATE, TIMEOUT_S, 0,1,0)){   // bus, baud, timeout, canonical, stop bits, parity 0=off 1=on.   ibus is no parity 1 stop bit.
			printf("Failed to rc_uart_init UART0\n");
//            return -1;
        }

	//Main Loop
	while (rc_get_state() != EXITING) {
		// do things based on the state
		
		if (rc_get_state() == RUNNING) {
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
		
			curms = rc_nanos_since_epoch()*0.000001;
//			printf("%X %X %X %X\n",datamain[0],datamain[1],datamain[2],datamain[3]);
			//curms = (int) (rc_nanos_since_epoch() * 0.000001);   // get current time for task timers
//			printf("curms is %" PRIu64 "\n",curms);   // remove after testing
		
// Continuous Tasks ============================================================================
					// run tasks required every loop

				// ---- Task for Receiver on UART0 -----
					if (rc_uart_bytes_available(RECEIVER) > 0) {     	// check uart0 buffer to see if any unread bytes are present
						ret = rc_uart_read_bytes(RECEIVER,_ch, 1);   	// read one byte from uart0 buffer into _ch
		//				printf("%X\n",chmain);
						if(ret<0) fprintf(stderr,"Error reading bus\n");
						else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
						else {
//							printf("Received %d bytes: %s \n", ret, _ch);
							ibus_read(_state,_data,chmain);  			// add byte from _ch to _data packet being built
						}				
					}

				
// 10ms Tasks ============================================================================
				if (curms-prev50ms >= 50) {
//					printf("time is %" PRIu64 "\n",curms-prev10ms);

					////Calculate Wheel Speed=============================================
					wheel_enc_data[0]= rc_encoder_read(q_encoder_pin[0]);
					wheel_enc_data[1] = rc_encoder_read(q_encoder_pin[1]);
					//curms = rc_nanos_since_epoch()*0.000001;
					vel[0] = (wheel_enc_data[0] - prev_wheel_enc_data[0])/(curms - prevE1ms);
					prevE1ms = curms;
					
					vel[1] = (wheel_enc_data[1] - prev_wheel_enc_data[1])/(curms - prevE2ms);
					prevE2ms = curms;
					prev50ms = curms;
					//=====================================================================
					// run 50ms tasks
					//encoder control		
					
					prev_wheel_enc_data[0] = wheel_enc_data [0];
					prev_wheel_enc_data[1] = wheel_enc_data [1];
					
					
					//printf("%f %f\n", vel[0], vel[1]);//prints wheel vel Rotation/ms
								

										
				}
					
				
// 40ms Tasks ============================================================================
				if (curms-prev100ms >= 100) {
					prev100ms = curms;
					
					/*
					for (int=0; i<5; i++){
						esc_ctrl = 
					}
					*/
					
					//Put input in a range of |0-500|
					rx_val[0] = (1500- datamain[0]);//maybe div by 334 to normalize to 1.5 (?)
					rx_val[1] = (1500- datamain[1]);
					//test motor
					
					
					///Test out ESC in 1 direction
					rc_servo_send_esc_pulse_normalized(1,0.9);
			
					
					
					
					// run 100ms tasks
					//Drive=======================================================					
					if (abs(rx_val[0])<50 && rx_val[1]<-50){//Reverse
						esc_ctrl[0] = rx_val[1];//should be negative
						esc_ctrl[1] = -1*rx_val[1];//should be positive
					}

					if(abs(rx_val[0])<50 && rx_val[1]>50){//Forward
						esc_ctrl[0] = rx_val[1];//should be positive
						esc_ctrl[1] = -1*rx_val[1];//should be negative
					}
					
					/*
					//Turn Logic
					if(datamain[0]>1550 && (datamain[1]<1550 && datamain[1]>1450)){
						esc_ctrl[0] = rx_val[1];//should be positive
						esc_ctrl[1] = 1*rx_val[1];//should be negative
					}
					
					if(datamain[1]>1550 && (datamain[0]<1550 && datamain[0]>1450)){
						rc_servo_send_pulse_normalized(esc_ch[0],esc_ctrl[0]);
						rc_servo_send_pulse_normalized(esc_ch[1],esc_ctrl[1]);
					}
					*/
					//===============================================================
					
					speed_ratio = vel[0]/vel[1];
					
					if (speed_ratio < 0){
						speed_ratio = -1.0*speed_ratio;
					}
					
					dt=prev100ms -curms;
					
					if(drive_state = 1){ //if bot is driving straight(drive_state will be set to 1): 5% Dead Zone
					
						//PID Control============================
						/*
					
					
						if(speed_ratio > 1.05 || speed_ratio< 0.95){
							if(speed_ratio>1){
								if(p=0){
									prev_error = 0;
								}
								p=1;
							}
						
							if(speed_ratio<1){
								if(p=1){
									prev_error = 0;
								}
								p=0;
							}
						
							error = 1 - speed_ratio;
							integral = integral + error * dt;
							derivative = (error - prev_error) / dt;
							w_output[p] = Kp * error + Ki * integral + Kd * derivative
							prev_error = error;
					
						}
						*/
						//=====================================
					
						//Basic Control: Increment slow side by 5%
					
						if(speed_ratio > 1.05 || speed_ratio< 0.95){
							if(speed_ratio < 1){
								esc_ctrl[0] = esc_ctrl[0]*1.05;
								if(abs(esc_ctrl[0])>500){
									esc_ctrl[1] = esc_ctrl[1]*0.95;//if control is maxxed out, decrease other motor (may have the possibility to accel to max after slight offset)
									esc_ctrl[0] = esc_ctrl[0]*0.95;//undo increment
								}
							}
							if(speed_ratio > 1){
								esc_ctrl[1] = esc_ctrl[1]*1.05;
								if(abs(esc_ctrl[1])>500){
									esc_ctrl[0] = esc_ctrl[0]*0.95;//if control is maxxed out, decrease other motor (may have the possibility to accel to max after slight offset)
									esc_ctrl[1] = esc_ctrl[1]*0.95;//undo increment
								}
							}
					
						}
						else{
							error=0;
							prev_error=0;
						}
						
					}
					//Add loop to iterate through channels(?)
					//normalize PPM, multiply -1 to to change turn, reduce number of needed servo commands/if statements(?)
					
					//Off Arm Encoder Logic=========================================================
					
					

					
					//Drive Commands======================================================
					
					///Prints ch1 and ch2
					printf("%d %d\n", datamain[0], datamain[1]);
					
					//UNCOMMENT The bottom two lines to control robot with TXRX
					
					//rc_servo_send_esc_pulse_normalized(esc_ch[0],(500+esc_ctrl[0])/1000);
					//rc_servo_send_esc_pulse_normalized(esc_ch[1],(500+esc_ctrl[1])/1000);
					//====================================================================
					
					///end Turn
					
					//Adds dead-zone to ch 1 - i
					//currently i max = 3
					
					/*
					for(int i=0;i<2;i++){
						
						if(datamain[i]<1550 && datamain[i]>1450){
							rc_servo_send_pulse_normalized(esc_ch[i],0);
						}
					}*/
				
				
				
				}
				
// 100ms Tasks ============================================================================				

				
		}
		else {
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// always sleep at some point
		rc_usleep(50);
	}
	printf("Closing UART 0\n");
	rc_uart_close(RECEIVER);
	// turn off LEDs and close file descriptors and shutdown Beagle
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();    // stop button handlers
	rc_remove_pid_file();   // remove pid file LAST
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	rc_dsm_cleanup();
	printf("Going for shutdown");
#ifdef BOT
	// system("sudo shutdown -P now");
#endif // BOT
	
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if (rc_get_state() == RUNNING)     rc_set_state(PAUSED);
	else if (rc_get_state() == PAUSED) rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for (i = 0; i < samples; i++) {
		rc_usleep(us_wait / samples);
		if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

void ibus_init(struct ibus_state *state, uint_fast8_t channel_count) {
  state->state = 0;
  state->channel_count = channel_count;
  return;
}

int ibus_read(struct ibus_state *state, uint16_t *data, uint8_t ch) {   
/* How ibus_read works:
You're building data[] to a full packet one byte at a time (one byte per call of ibus_read).
A full packet from receiver is 0x20 0x40 CH0 CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10 CH11 CH12 CH13 CHKSUM.  Each CH and CHKSUM is 2 bytes making packet 1+1+(2*14)+2 = 32 bytes long stored in data[].
The transmitter will send a new packet every 7ms until a few tenths of a second after the transmitter is switched off.
The checksum is calculated by starting at 0xFFFF then subtracting each byte except the checksum bytes.

For Team 8 we will utilize as follows:
  data[0]	CH0		Channel 1 - Right Horizontal 		- Left/Right Turn - ranges 3E8 to 7D0
  data[1]	CH1		Channel 2 - Right Vertical 			- Forward/Reverse - ranges 3E8 to 7D0
  data[2]	CH2		Channel 3 - Left Vertical 			- TBD Usage - ranges 3E8 to 7D0
  data[3]	CH3		Channel 4 - Left Horizontal 		- TBD Usage - ranges 3E8 to 7D0
  data[4]	CH4		Channel 5 - 1st switch - SWA 		- TBD Usage - ranges 3E8 to 7D0   ** may have this mixed up and is actually POT1
  data[5]	CH5		Channel 6 - 2nd switch - SWB 		- TBD Usage - ranges 3E8 to 7D0   ** may have this mixed up and is actually POT2
  data[6]	CH6		Channel 7 - 3rd switch - POT1 		- TBD Usage - ranges 3E8 to 7D0   ** may have this mixed up and is actually SWA
  data[7]	CH7		Channel 8 - 4th switch - POT2 		- TBD Usage - ranges 3E8 to 7D0   ** may have this mixed up and is actually SWB
  data[8]	CH8		Channel 9 - 5th switch - SWC 		- TBD Usage - ranges 3E8 to 7D0
  data[9]	CH9		Channel 10 - 6th switch - SWD 		- TBD Usage - ranges 3E8 to 7D0
  data[10]	CH10	Channel 11 							- Unused - always 5DC
  data[11]	CH11	Channel 12 							- Unused - always 5DC
  data[12]	CH12	Channel 13 							- Unused - always 5DC
  data[13]	CH13	Channel 14 							- Unused - always 5DC
  
  Unassigned switch ideas:
	motors/escs  enable/disable (all motors off safety switch)
	self-righting switch
	clockwise spin mode switch
	arm calibration pot
	speed limit pot
	
   
Details how ibus_read works:
after ibus_init, state->state is at 0.  
in state 0 will only look for arg3 to be 0x20....once arg3 is 0x20 state changes to 1.
in state 1 will only look for arg3 to be 0x40....once arg3 is 0x40 state changes to 2.
in state 2 goes to default....channel count is 10 so state/2=1 which is <=10....state 2 & 1 (is it even?) is true so set data1=ch and increment state
in state 3 goes to default....channel count is 10 so state/2=1 which is <=10....state 3 & 1 (is it even?) is false so set data[0]=ch||data1 and increment state
in state 22 goes to default....channel count is 10 so state/2=11 which is not <=10 so only increment state
(in state 29 you'd be storing data[13] which is the maximum number)
in state 30 you store the byte in data1 and set state to 31
in state 31 you set checksum to ch || data1 and compare that to your local calculated checksum
*/
  switch (state->state) {
  case 0:
    if (ch == 0x20) {
      state->checksum = 0xFFFF - 0x20;
      state->state = 1;
    }
    break;
  case 1:
    if (ch == 0x40) {
      state->state = 2;
      state->checksum -= ch;
    } else {
      // Unknown packet type
      state->state = 0;
    }
    break;
  case 30:
    state->datal = ch;
    state->state = 31;
    break;
  case 31: {
    uint_fast16_t checksum = (ch << 8) | state->datal;
    state->state = 0;
    if (checksum == state->checksum)
      return 0;
  } break;
  default:
    // Ignore these bytes if we've filled all of the channels
    if (state->state / 2 <= state->channel_count) {
      if ((state->state & 1) == 0) {
        // Data low byte
        state->datal = ch;
      } else {
        // Data high byte
        data[(state->state / 2) - 1] = (ch << 8) | state->datal;
      }
    }
    state->checksum -= ch;
    ++state->state;
    break;
  }

  return -1;
}

// Prints all channel data
// [Ch1] [Ch2] [Ch3] [Ch4] [Ch5] [Ch6] [Ch7] [Ch8] [Ch9] [Ch10]

/* void print_data(uint16_t *ibus_data, int ch){
	int i=0;
	
	if (ch>10){
		printf("Invalid Channel Number. Channel %d does not exist.\n",ch);
	}else if(ch == 10){
		for( i=0; i<9;i++){
			printf("[%d] ",ibus_data[i]);
		}
		printf("[%d]\n",ibus_data[i]);
	}else{
		printf("%d  \n",ibus_data[ch]);
	}
	
} */

/* double interp(uint16_t ch1_in, uint16_t ch2_in){
	
} */

/*
//Function to take raw joystick input and blend to tank steer commands
void drive_values(){
	/*Notes
		Ch2 = Y axis = datamain[1]
		Ch1 = X axis = datamain[0]
		
		Missing variables
			left_in
			right_in
			left_out
			right_out
			temp_RO
			limiter
			constrain function
	//
	left_in = ch1_in;
	right_in = ch2_in;
	left_out = ch1_in;
	right_out = ch2_in;
  
	if(reverseDrive == true){
		int temp_RO = right_out;
		right_out = (1500 + ((left_out - 1500) * -1));
		left_out = (1500 + ((temp_RO - 1500) * -1));
	}
	
	switch(limiter){
		case 25:
			right_out = constrain(right_out, 1375, 1625);
			left_out = constrain(left_out, 1375, 1625);
			break;
		case 50:
			right_out = constrain(right_out, 1250, 1750);
			left_out = constrain(left_out, 1250, 1750);
			break;
		case 75:
			right_out = constrain(right_out, 1125, 1875);
			left_out = constrain(left_out, 1125, 1875);
			break;
  }
	
}
*/
