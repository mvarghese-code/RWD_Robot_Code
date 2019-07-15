/*
 * @file rc_test_encoders.c
 *
 * @example    rc_test_encoders
 *
 * Prints out current quadrature position for channels 1-4. 1-3 are counted
 * using the eQEP modules. Channel 4 is counted by the PRU.
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <rc/encoder.h>
#include <rc/time.h>
#include <rc/i2c.h>

//Must add i2c bus and address to 
#DEFINE I2C_BUS 2

//Address of offensive arm encoder
#DEFINE ENCODER_ADD


static int running = 0;
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}
int main()
{
		uint64_t prev10ms = 0;  	// init task timers
		uint64_t prev50ms = 0;		// init task timers
		uint64_t prev100ms = 0;		// init task timers
		uint64_t prevE1ms = 0;		// init task timers
		uint64_t prevE2ms = 0;		// init task timers
		uint64_t curms; 			// init task timers
        //PID Variables=====================
		double integral = 0.0;
		double derivative = 0.0;
		double Kp, Kd, Ki = 0.0, 0.0, 0.0;
		double error, prev_error =0.0, 0.0;
		//==================================
		
		int i=0;
		int vel[2] ={0,0};
		int wheel_enc_data[2] = {0,0};
		int prev_wheel_enc_data[2] = {0,0};
		uint8_t q_encoder_pin[2]= {4,4};
		uint8_t i2c_data = 0;
		double speed_ratio = 0.0;
		
		
        // initialize hardware first
        if(rc_encoder_init()){
                fprintf(stderr,"ERROR: failed to run rc_encoder_init\n");
                return -1;
        }
		
		//initialize i2c bus and device
		if(rc_i2c_init()){
                fprintf(stderr,"ERROR: failed to run rc_i2c_init\n");
                return -1;
        }
        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running=1;
        printf("\nRaw encoder positions\n");
        printf("      E3R   |");
        printf("      E4L   |");
        printf("      EOff   |");
        printf(" \n");
        while(running){
				curms = rc_nanos_since_epoch()*0.000001;
				// 50ms Tasks ============================================================================
				if (curms-prev50ms >= 50) {
					////Calculate Wheel Speed
					rc_encoder_read(q_encoder_pin[i])
					curms = rc_nanos_since_epoch()*0.000001;
					vel[0] = (wheel_enc_data[0] - prev_wheel_enc_data[0])/(curms - prevE1ms);
					prevE1ms = curms;
					
					vel[1] = (wheel_enc_data[1] - prev_wheel_enc_data[1])/(curms - prevE2ms);
					prevE2ms = curms;
					prev50ms = curms;
					// run 50ms tasks
					//encoder control			

										
				}
				
				if (curms-prev100ms >= 100){
				// 100ms Tasks ============================================================================
					speed_ratio = vel[0]/vel[1];
					
					dt=prev100ms -curms;
					
					if(drive_state = 1){ //if bot is driving straight: 5% Dead Zone
					
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
								//increase left side by 10%
							}
							if(speed_ratio > 1){
								//increase right side by 10%
							}
					
						}
					}else{
						error=0;
						prev_error=0;
					}
					//======================================
				
				prev100ms = curms;
				}
				
                printf("\r");
				
				//Read and print quad encoder data
                for(i=1;i<=2;i++){
                        printf("%10d |", rc_encoder_read(q_encoder_pin[i]));
                }
			
				//Read and print i2c data
				/*
				rc_i2c_read(I2C_BUS,ENCODER_ADD, *i2c_data);
				printf("%10d |", i2c_data);
				*/
                fflush(stdout);
                rc_usleep(50000);
        }
        printf("\n");
		rc_i2c_close(I2C_BUS);
        rc_encoder_cleanup();
        return 0;
}