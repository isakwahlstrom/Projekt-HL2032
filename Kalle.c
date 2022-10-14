/* Including packages and files */ 
#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "gd32vf103v_eval.h"
#include "stdio.h"
#include "string.h"
#include "gd32v_tf_card_if.h"
#include "pwm.h"'
#include "math.h"

/* Define global variables */
#define GRAPH_HEIGHT 30
#define EI 1
#define DI 0

#define PWMC1_PORT     GPIOA
#define PWMC1_PIN      GPIO_PIN_1
#define PWM_CHANNEL   

/* Create a package struct to transmit (transmit and recive data from gyro/acc, vectors) */
struct Package {
    float aX,aY,aZ,gX,gY,gZ;
};

////////////////////////// Define functions /////////////////////////////////////////
void Initialize_Project();


void Recive_IMU_Data(struct Package *package);
void Send_IMU_Data(struct Package package);

void HapticFeedback(float x, float y);

void SendToSD(float data[]);

void init_PWM_example();

unsigned int int_sqrt ( unsigned int s );
unsigned int convert_int_to_float ( float f);
/* Main function */

int main(void) {
    Initialize_Project();

	///////////////////////// Acc & Gyro variables ///////////////////////////////////

	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    //float ba, bo, sa, so;
    //ba = 22; bo = 22; sa = 33; so = 33;
    mpu_vector_t Acc, Gyro;

	//////////////////////////////////// Usart variables //////////////////////////////////////
	
    struct Package package;
    struct Package package2;

    /////////////////////////////////// Math variables ///////////////////////////////////////// 
    float Ax, Ay, Gx, Gy;
    float PosX, PosY;
    //float StartX, StartY;
    //float LatestX, LatestY;

    /////////////////////////////// Haptic-Feedback variables ///////////////////////////////////// 

    /* Hardware PWM on the GD32Vf103 is generted with the help of a timer. It basically compares a number in a register against the current timer count
       and sets the state of the pin low or high based on if the channels value is higher or lower than the current count. */

    /* This example displays a pulsing light in a triangle wave pattern */

    int32_t duty = 0;
    int32_t diff = 4096/128;

    /* Initialize gpio for alternate function */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    /* First we need to set up the timer */
    init_PWM_example();

    /////////////////////////////////// SD-Card variables ///////////////////////////////////////// 
    
    /* Create array for transmiting data to SD-Card & Clock to see when we should send it */
    float data[128] = {11.11,22.22,11.11,33.33,11.11,44.44,11.11,55.55,11.11,66.66};
    int clock = 0;
	
    
    
    /* Infinity while loop to always check gyro/acc & sedn/recive */
	while(1) {
        ///////////////////////////////////////////////// Accel & Gyro //////////////////////////////////////////////////////
		
        mpu6500_getGyroAccel(&Acc,&Gyro);
        // Calls on getAccel and getGyro at the same time
        package.aX = Acc.x/4096;
        package.aY = Acc.y/4096;
        package.aZ = Acc.z/4096;
        package.gX = Gyro.x/16.4;
        package.gY = Gyro.y/16.4;
        package.gZ = Gyro.z/16.4;

        /////////////////////////////////////////////////// USART ////////////////////////////////////////////////////////////  
        Recive_IMU_Data(&package2);
        //float num =  *((float*)buffer);

        /////////////////////////////////////////////////// Math //////////////////////////////////////////////////////////// 
        // 2D Matrix: https://beginnersbook.com/2014/01/2d-arrays-in-c-example/
        int room[500][500] = 0;
        int StartX = 250, StartY = 250;
        room[StartX][StartY] = 1;
        int LatestX, LatestY;
        // 3D Matrix: https://owlcation.com/stem/How-to-work-with-Multidimensional-Array-in-C-Programming
        Ax = package2.aX - package.aX;
        Ay = package2.aY - package.aY;
        Gx = package2.gX - package.gX;
        Gy = package2.gY - package.gY;
        
        PosX += Ax*Gx;
        PosY += Ay*Gy;
        // PosX = ba * bo;
        // PosY = sa * so;
        
        /* Skriv kod för att navigera i ett 2D rutnät */
        room[LatestX][LatestY] = 0;

        LatestX += StartX + int_sqrt((package.aX*package.aX));
        LatestY += StartY + int_sqrt((package.aY*package.aY));

        room[LatestX][LatestY] = 1;
        // https://en.wikipedia.org/wiki/Integer_square_root
        // https://www.vedantu.com/maths/magnitude-of-a-vector
        // Acc:
        // The vector R is the force vector that the accelerometer is measuring (it could be either the gravitation force or the inertial force from the examples above or a combination of both).
        // R^2 = Rx^2 + Ry^2 + Rz^2

        // https://robotics.stackexchange.com/questions/18446/how-to-transform-raw-accelerometer-data-into-the-earth-fixed-frame-to-determine
        // https://www.i2cdevlib.com/forums/topic/4-understanding-raw-values-of-accelerometer-and-gyrometer/
        // https://www.instructables.com/Accelerometer-Gyro-Tutorial/
        // I 2D vill vi använda oss av Euler angle för att räkna ut en ungefärlig orientering från gyroskopet. Integrering av gyro-datan.
        // När vi sedan går över till 3D vill vi använda Tait-Bryan angles.
        // Vi vill använda Eigen biblioteket för att convertera accelerometer matriser ?


        /////////////////////////////////////////////////// Haptic-Feedback //////////////////////////////////////////////////////////// 
        /* Update pulse width*/
        timer_channel_output_pulse_value_config(TIMER4,TIMER_CH_1,(int)duty);
        
        /* Create triangle wave */
        duty += diff; 

        if(duty > 4096 || duty < 0) diff = -diff; //If the full duty cycle is reached start counting down, if below zero start counting up
        if(duty < 0) duty = 0;                    //Make sure no negative values get written as the dutycycle

        /* Wait a short moment */
        delay_1ms(6);

        /////////////////////////////////////////////////// SD-Card //////////////////////////////////////////////////////////// 
        data[clock] = PosX;
        data[clock + 1] = PosY;   
        if (clock == 10){
            //SendToSD(data);
            clock = 0;
            //LCD_ShowChar(50,50,'X',TRANSPARENT,GREEN);
        }
    
        delay_1ms(1000);
        clock += 2;
	    LCD_Clear(1);
    };
}

////////////////////////// Write functions /////////////////////////////////////////

void Initialize_Project(){
    ////////////////////////////////////// Initialize LCD ///////////////////////////////////////////
	Lcd_SetType(LCD_INVERTED);
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

	//////////////////////////////////// Initialize Usart ///////////////////////////////////////////

	/* USART interrupt configuration */
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(USART0_IRQn, 1, 0);
		
    /* configure COM0 */
    gd_eval_com_init(EVAL_COM0);                                // Startar hela USART systemet.

    /////////////////////////////////// Initialize Math ///////////////////////////////////////////// 

    /////////////////////////////// Initialize Haptic-Feedback ///////////////////////////////////// 

    /////////////////////////////////// Initialize SD-Card ///////////////////////////////////////////// 

}


void Recive_IMU_Data(struct Package *package){
    uint8_t rx_size = 32;
    __IO uint16_t rxcount = 0;
    uint8_t * rxbuffer = (uint8_t*)&package;
    usart_interrupt_enable(USART0, USART_INT_RBNE);

    while(rxcount<sizeof(package)) {
            if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){ 
                rxbuffer[rxcount++] = usart_data_receive(USART0);
            }
        }
        rxcount=0;
    
}

void Send_IMU_Data(struct Package package){
    uint8_t tx_size = 32;
    __IO uint8_t txcount = 0; 
    uint8_t * txbuffer = (uint8_t*)&package;
    usart_interrupt_enable(USART0, USART_INT_TBE);

    while(txcount < sizeof(package)) {
            if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){ 
                usart_data_transmit(USART0, txbuffer[txcount++]);
            }
        }
        txcount =0;

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

void SendToSD(float data[]){
    FATFS fs;
    volatile FRESULT fr;
    FIL file;

    UINT bw = 0;

    char information[128];
    int integerX, integerY, decimalX, decimalY;
    
    set_fattime(2022,10,12,0,0,0);
    delay_1ms(100);
    
    strcpy(information,"");
    for (int i = 0; i < 10; i+=2){
        sprintf(&information[strlen(information)], "%02f / %02f | ", data[i], data[i+1]);
    }
    strcat(information,"");
    
    f_mount(&fs,"",1);
    f_sync(&file);

    fr = f_open(&file, "TEST.TXT", FA_WRITE | FA_OPEN_APPEND);
    fr = f_write(&file, information, strlen(information), &bw);
    delay_1ms(400);

    f_sync(&file);

    f_close(&file);
    delay_1ms(100);

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

// Square root of integer
unsigned int int_sqrt ( unsigned int s )
{
	// Zero is always yields zero
	if (s == 0) 
		return 0;

	unsigned int x0 = s / 2;				// Initial estimate

	unsigned int x1 = ( x0 + s / x0 ) / 2;	// Update
		
	while ( x1 < x0 )						// Bound check
	{
		x0 = x1;
		x1 = ( x0 + s / x0 ) / 2;
	}		
	return x0;
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
