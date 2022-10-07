/* Including packages and files */ 
#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include <stdio.h>
#include "gd32vf103v_eval.h"
// #include "gd32vf103_it.h"

/* Define global variables */
#define GRAPH_HEIGHT 30
#define EI 1
#define DI 0

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE            (ARRAYNUM(txbuffer) - 1)

/* Create variables for transmit & recive */
uint8_t txbuffer[] = "brassi";
uint8_t rxbuffer[32];
uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = TRANSMIT_SIZE;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0;

/* Create a package struct to transmit (transmit and recive data from gyro/acc, vectors) */
struct Package {
    float aX, aY, aZ, gX, gY, gZ;
} Package;

uint8_t buffer[sizeof(Package) + 1];

// Antingen skcikar vi denna buffert lite i taget, med pointers från och till uint8_t alternativt använder union till en "vanlig array"

/* Main function */

int main(void) {
	////////////////////////// Initialize LCD /////////////////////////////////////////
    
	Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(BLACK);

	/* int to write T/R characters on LCD screen */
	int x = 0;
	int y = 0;
	int x2 = 80;
	int y2 = 0;

	///////////////////////// Initialize Acc & Gyro ///////////////////////////////////

	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t Acc, vec_temp;
    mpu_vector_t Gyro, vec2_temp;
    /* for lcd */
    uint16_t line_color;

	/* Initialize pins for I2C */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */
    mpu6500_install(I2C0);

	/* Creates temporary vectors to erase previously drwan lines */
    mpu6500_getAccel(&vec_temp);
    mpu6500_getGyro(&vec2_temp);

	//////////////////////////////////// Initialize Usart //////////////////////////////////////

	/* USART interrupt configuration */
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(USART0_IRQn, 1, 0);
		
    /* configure COM0 */
    gd_eval_com_init(EVAL_COM0);                                // Startar hela USART systemet.
	

	/* Infinity while loop to always check gyro/acc & sedn/recive */
    
	//while(1) {
        
        ///////////////////////////////////////////////// Acc & Gyro //////////////////////////////////////////////////////////
		
        // Calls on getAccel and getGyro at the same time
        mpu6500_getGyroAccel(&Acc,&Gyro);

        ////////////////////////////////////////////// Pack package //////////////////////////////////////////////////////////// 

        Package.aX = Acc.x;
        Package.aY = Acc.y;
        Package.aZ = Acc.z;
        Package.gX = Gyro.x;
        Package.gY = Gyro.y;
        Package.gZ = Gyro.z;


        /////////////////////////////////////////////////// USART ////////////////////////////////////////////////////////////////  
        
        /* enable USART TBE & RBNE interrupt */  
        usart_interrupt_enable(USART0, USART_INT_TBE);
        usart_interrupt_enable(USART0, USART_INT_RBNE);
        
        /* Send txbuffer and recive it to save in rxbuffer */
        for (int i = 0; i < rx_size; i++){
            LCD_ShowChar(x+=10,40,rxbuffer[i],TRANSPARENT,GREEN);
            LCD_Wait_On_Queue();
        }
        x=0;
        while((txcount < tx_size) && (rxcount < rx_size)) {
            if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){ 
                usart_data_transmit(USART0, txbuffer[txcount++]);
                LCD_ShowNum(80,10,txcount,2,BLUE);
                LCD_Wait_On_Queue();
            }
            delay_1ms(250);
            if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){ 
                rxbuffer[rxcount++] = usart_data_receive(USART0);
                LCD_ShowNum(100,10,rxcount,2,YELLOW);
                LCD_Wait_On_Queue();
            }
        }

        /* Check that everything has been sent and everything has been recived */ 
        if(rxcount == rx_size) {
            LCD_ShowChar(10,10,'O',TRANSPARENT,RED);
            LCD_Wait_On_Queue();
        }
        
        if(rxcount == rx_size) {
            LCD_ShowChar(20,10,'K',TRANSPARENT,RED);
            LCD_Wait_On_Queue();
        }

        /* Print out rxbuffer, the recived information to see that everything is there */
        for (int i = 0; i < rx_size; i++){
            LCD_ShowChar(x+=10,60,rxbuffer[i],TRANSPARENT,GREEN);
            delay_1ms(250);
        }
		
    //};
}
