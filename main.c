/* Including packages and files */ 
#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "gd32vf103v_eval.h"
#include "stdio.h"
#include "string.h"
#include "gd32v_tf_card_if.h"
#include "pwm.h"

#define PWMC1_PORT     GPIOA
#define PWMC1_PIN      GPIO_PIN_1
#define PWM_CHANNEL   

////////////////////////// Define functions /////////////////////////////////////////
void Initialize_Project();

void HapticFeedback(float x, float y);

void SendToSD(int data);

void init_PWM_example();

unsigned int convert_int_to_float ( float f);

///////////////////////////// Main function /////////////////////////////////////////
int main(void) {
    Initialize_Project();

	///////////////////////// Acc & Gyro variables ///////////////////////////////////

	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t Acc, Acc2;
    float x, y, z;
    int on = 0;

    /////////////////////////////// Haptic-Feedback variables ///////////////////////////////////// 

    /* Hardware PWM on the GD32Vf103 is generted with the help of a timer. It basically compares a number in a register against the current timer count
       and sets the state of the pin low or high based on if the channels value is higher or lower than the current count. */

    /* This example displays a pulsing light in a triangle wave pattern */
    int32_t duty = 0;
    int32_t diff = 4096/128;

    /////////////////////////////////// SD-Card variables ///////////////////////////////////////// 
    
    /* Create an int for transmiting data to SD-Card & Clock to see when we should send it */
    int redReps = 0;
    int period = 7200;
    uint64_t start_mtime, delta_mtime;
    
    /* Infinity while loop to always check Acc, gives feedback and sends info to SD-card */
	while(1) {
        ///////////////////////////////////////////////// Accel & Gyro //////////////////////////////////////////////////////
		// Read button pressed (GPIO-pin)
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_6)){          // if Button is pressed                    
            while(gpio_input_bit_get(GPIOA, GPIO_PIN_6));   // while Button is pressed
            if(on == 0)
            {
                on = 1;
            } else {
                on = 0;
            }
        }

        // Change state between break and work
        if(on == 0)
        {
            /* On a break */
            start_mtime = get_timer_value();
        } else {
            // Get accelleration data (Note: Blocking read) puts a force vector with 1G = 4096 into x, y, z directions respectively
            mpu6500_getAccel(&Acc);

            /* Add information from Acc2, our IMU on the wrist */

            // Scale to G values
            x = Acc.x / 16384;
            y = Acc.y / 16384;
            z = Acc.z / 16384;

            // Convert to positive
            if (x < 0) {
                x = x*(-1);
            }
            if (z < 0) {
                z = z*(-1);
            }

            // Compare y to the gravitational pull, y is higher then 0.8 G we're in the Green zone
            if(y>=0.8)  {
                LCD_Clear(GREEN);
            } else {
                while(y<0.8)  {
                    LCD_Clear(RED);
                }
                redReps += 1;
            }

            // OR......

            // Compare x and z to the gravitational pull, if x is higher then 0.2 G or y is higher then 0.3 G we're in the Red zone
            // if((x>=0.2) || ((z>=0.3)))  {
                    // while((x<0.2) || ((z<0.3))){
                    //     LCD_Clear(RED);
                    // }
                    // redReps += 1;
            // } else {
            //     LCD_Clear(GREEN);
            // }
        }

        /////////////////////////////////////////////////// Haptic-Feedback ////////////////////////////////////////////////////////////         
        // Give haptic feedback every 10th repetition
        if((redReps%10)==0)
        {
            /* Send vibration ... Or.... Light LED */

            /* Update pulse width*/
            timer_channel_output_pulse_value_config(TIMER4,TIMER_CH_1,(int)duty);

            /* Create triangle wave */
            duty += diff; 

            if(duty > 4096 || duty < 0) diff = -diff; //If the full duty cycle is reached start counting down, if below zero start counting up
            if(duty < 0) duty = 0;                    //Make sure no negative values get written as the dutycycle

            /* Wait a short moment */
            delay_1ms(6);
        }

        /////////////////////////////////////////////////// SD-Card //////////////////////////////////////////////////////////// 
        // Count up the time passed in ms
        delta_mtime = get_timer_value() - start_mtime;

        // Send to SD-Card after 2 hours of work
        if (delta_mtime >(SystemCoreClock/4000.0 *period)){
            SendToSD(redReps);      // Send redReps to SD-Card
        }
    };
}

////////////////////////// Write functions /////////////////////////////////////////

void Initialize_Project(){
    ////////////////////////////////////// Initialize LCD ///////////////////////////////////////////
	Lcd_SetType(LCD_NORMAL);
    Lcd_Init();
    LCD_Clear(1);
    LCD_DrawPoint(1,1,1);
    delay_1ms(100);

	//////////////////////////////////// Initialize Acc & Gyro /////////////////////////////////////

	/* Initialize pins for I2C */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */
    mpu6500_install(I2C0);

    rcu_periph_clock_enable(RCU_GPIOA);
    /* This configures the A3 and A4 pins as inputs with internal pull ups enabled */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /////////////////////////////// Initialize Haptic-Feedback ///////////////////////////////////// 
    /* Initialize gpio for alternate function */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    /* First we need to set up the timer */
    init_PWM_example();
}

void init_PWM_example(){

    /* These structs are used for configuring the timer */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    /* First we need to enable the clock for the timer */
    rcu_periph_clock_enable(RCU_TIMER4);

    /* Reset the timer to a known state */
    timer_deinit(TIMER4);

    /* This function sets the struct up with default values */
    timer_struct_para_init(&timer_initpara);

    /* timer configuration */
    timer_initpara.prescaler         = 1;                   // Prescaler 1 gives counter clock of 108MHz/2 = 54MHz 
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;  // count alignment edge = 0,1,2,3,0,1,2,3... center align = 0,1,2,3,2,1,0
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;    // Counter direction
    timer_initpara.period            = 4095;                // Sets how far to count. 54MHz/4096 = 13,2KHz (max is 65535)
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;    // This is used by deadtime, and digital filtering (not used here though)
    timer_initpara.repetitioncounter = 0;                   // Runs continiously
    timer_init(TIMER4, &timer_initpara);                    // Apply settings to timer


    /* This function initializes the channel setting struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* PWM configuration */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;                   // Channel enable
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;                 // Disable complementary channel
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;             // Active state is high
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;    
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;            // Idle state is low
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER4,TIMER_CH_1,&timer_ocinitpara);   // Apply settings to channel

    timer_channel_output_pulse_value_config(TIMER4,TIMER_CH_1,0);                   // Set pulse width
    timer_channel_output_mode_config(TIMER4,TIMER_CH_1,TIMER_OC_MODE_PWM0);         // Set pwm-mode
    timer_channel_output_shadow_config(TIMER4,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER4);

    /* start the timer */
    timer_enable(TIMER4);
}


void HapticFeedback(float x, float y){
    int value = 0;
    int max = 100;
    if ((x > max) || (y > max)){
        value = 1;
    } else {
        value = 0;
    }
    T1setPWMmotorB(value);
}

void SendToSD(int data){
    FATFS fs;
    volatile FRESULT fr;
    FIL file;

    UINT bw = 0;

    char information[128];
    
    set_fattime(2022,10,12,0,0,0);
    delay_1ms(100);
    
    strcpy(information,"");
    sprintf(&information[strlen(information)], "%d", data);
    strcat(information,"");
    
    f_mount(&fs,"",1);
    f_sync(&file);

    fr = f_open(&file, "DATA.TXT", FA_WRITE | FA_OPEN_APPEND);
    fr = f_write(&file, information, strlen(information), &bw);
    delay_1ms(400);

    f_sync(&file);

    f_close(&file);
    delay_1ms(100);

}

unsigned int convert_int_to_float ( float f) {
    int integer, decimal;
    integer = f;
    decimal = (f - integer ) * 10;
    if (decimal >= 5){
        integer++;
    }
    return integer;
}
