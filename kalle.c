/* Including packages and files */ 
#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "gd32vf103v_eval.h"
#include "stdio.h"
#include "string.h"
#include "gd32v_tf_card_if.h"

/* Define global variables */
#define GRAPH_HEIGHT 30
#define EI 1
#define DI 0

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE            (ARRAYNUM(txbuffer) - 1)

/* Create variables for transmit & recive */
uint8_t txbuffer[] = "brassi";
//uint8_t rxbuffer;
uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = TRANSMIT_SIZE;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0;

/* Create a package struct to transmit (transmit and recive data from gyro/acc, vectors) */
struct Package {
    float aX,aY,aZ,gX,gY,gZ;
};

float data[128] = {11.11,22.22,11.11,33.33,11.11,44.44,11.11,55.55,11.11,66.66};
int clock = 0;

void HepticFeedback(float x, float y);

void SendToSD(float data[]);

/* Main function */

int main(void) {
	////////////////////////// Initialize LCD /////////////////////////////////////////
	Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(1);
    LCD_DrawPoint(1,1,1);
    delay_1ms(100);
     
	/* int to write T/R characters on LCD screen */
	int x = 0;
	int y = 0;
	int x2 = 80;
	int y2 = 0;

	///////////////////////// Initialize Acc & Gyro ///////////////////////////////////

	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    float ba, bo, sa, so;
    ba = 22; bo = 22; sa = 33; so = 33;
    mpu_vector_t Acc, Acc_temp;
    mpu_vector_t Gyro, Gyro_temp;

	/* Initialize pins for I2C */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */
    mpu6500_install(I2C0);

	//////////////////////////////////// Initialize Usart //////////////////////////////////////

	/* USART interrupt configuration */
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(USART0_IRQn, 1, 0);
		
    /* configure COM0 */
    gd_eval_com_init(EVAL_COM0);                                // Startar hela USART systemet.
	
    
	/* enable USART TBE & RBNE interrupt */  
    usart_interrupt_enable(USART0, USART_INT_TBE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);

    struct Package package;
    
    struct Package package2;
    
    uint8_t * rxbuffer = (uint8_t*)&package2;
    uint8_t * bufferstruct = (uint8_t*)&package;


    /////////////////////////////////// Initialize Math ///////////////////////////////////////// 
    float PosX, PosY;
    //float StartX, StartY;
    //float LatestX, LatestY;
    /*  */


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

        while((txcount < sizeof(package) && rxcount<sizeof(package))) {
            if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){ 
                usart_data_transmit(USART0, bufferstruct[txcount++]);
            }
            //delay_1ms(50);
            if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){ 
                rxbuffer[rxcount++] = usart_data_receive(USART0);
            }
        }
        txcount =0;
        rxcount=0;

        //float num =  *((float*)buffer);

        /////////////////////////////////////////////////// Math //////////////////////////////////////////////////////////// 
        PosX = ba * bo;
        PosY = sa * so;
        /*
        StartX = 0; 
        StartY = 0;
        LatestX = package2.aX - StartX;
        LatestY = package2.aY - StartY;
        if(LatestX < 0){
            if(LatestY < 0){
                LCD_Clear(GREEN);
            } else if(LatestY > 0){
                LCD_Clear(RED);
            }
        } else if(LatestX > 0){
            if(LatestY < 0){
                LCD_Clear(YELLOW);
            } else if(LatestY > 0){
                LCD_Clear(BLUE);
            }
        }
        Skriv kod för att navigera i ett 2D rutnät */

     /////////////////////////////////////////////////// SD-Card //////////////////////////////////////////////////////////// 
    data[clock] = PosX * clock;
    data[clock + 1] = PosY * clock;   
    if (clock == 10){
        SendToSD(data);
        clock = 0;
        LCD_ShowChar(50,50,'X',TRANSPARENT,GREEN);
    }
    
    delay_1ms(1000);
    clock += 2;
	LCD_Clear(1);
    };
}

void HepticFeedback(float x, float y){
    int value = 0;
    int max = 100;
    if ((x > max) || (y > max)){
        value = 1;
    } else {
        value = 0;
    }
    //T1setPWMotorB(value);
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
