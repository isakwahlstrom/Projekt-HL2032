/* Including packages and files */ 
#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "gd32vf103v_eval.h"
#include "stdio.h"

/* Define global variables */
#define GRAPH_HEIGHT 30
#define EI 1
#define DI 0

/* Create a package struct to transmit (transmit and recive data from gyro/acc, vectors) */
struct Package {
    float aX,aY,aZ,gX,gY,gZ;
};

////////////////////////// Define functions /////////////////////////////////////////

void Send_IMU_Data(struct Package package);

void HapticFeedback(float x, float y);

/* Main function */

int main(void) {
    Initialize_Project();

    ///////////////////////// Variables ///////////////////////////////////

    /* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t Acc, Gyro;
	
    struct Package package;
    
    /* Infinity while loop to always check gyro/acc & sedn/recive */
	while(1) {
        ///////////////////////////////////////////////// Accel & Gyro //////////////////////////////////////////////////////
		
        mpu6500_getGyroAccel(&Acc,&Gyro);
        // Calls on getAccel and getGyro at the same time
        package.aX = Acc.x;
        package.aY = Acc.y;
        package.aZ = Acc.z;
        package.gX = Gyro.x;
        package.gY = Gyro.y;
        package.gZ = Gyro.z;

        /* Skala värdena!!! */

        /////////////////////////////////////////////////// USART ////////////////////////////////////////////////////////////  
        Send_IMU_Data(package);

        /////////////////////////////////////////////////// Haptic-Feedback //////////////////////////////////////////////////////////// 

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

    /////////////////////////////// Initialize Haptic-Feedback ///////////////////////////////////// 

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
    //T1setPWMotorB(value);
}
