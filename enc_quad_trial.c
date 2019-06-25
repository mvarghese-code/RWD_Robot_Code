/*
 * @file rc_test_encoders.c
 *
 * @example    rc_test_encoders
 *
 * Prints out current quadrature position for channels 1-4. 1-3 are counted
 * using the eQEP modules. Channel 4 is counted by the PRU.
 */
#include <stdio.h>
#include <signal.h>
#include <rc/encoder.h>
#include <rc/time.h>
#include <rc/i2c.h>

//Must add i2c bus and address to 
#define I2C_BUS 1

//Address of offensive arm encoder
#define ENCODER_ADD 0x38


static int running = 0;
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}
int main()
{
        int i;
		uint8_t q_encoder_pin[2]= {4,4};
		uint8_t i2c_data = 0;
		uint16_t r_vel = 0;
		uint16_t l_vel = 0;
		uint32_t r_enc = 0;
		uint32_t l_enc = 0;
		
		uint64_t prev10ms = 0;  	// init task timers
		uint64_t prev40ms = 0;		// init task timers
		uint64_t prev100ms = 0;		// init task timers
		uint64_t curms; 			// init task timers
		
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
        printf("      E3   |");
        printf("      E4   |");
        printf("      EOff   |");
        //printf("      E4   |");
        printf(" \n");
        while(running){
                printf("\r");
				curms = rc_nanos_since_epoch()*0.000001;
				
                for(i=1;i<=4;i++){
						
                        printf("%10d |", rc_encoder_read(q_encoder_pin[i]));
                }
			
				//rc_i2c_read(I2C_BUS,ENCODER_ADD, i2c_data);
				//printf("%10d |", i2c_data);
                fflush(stdout);
                rc_usleep(50000);
        }
        printf("\n");
		rc_i2c_close(I2C_BUS);
        rc_encoder_cleanup();
        return 0;
}