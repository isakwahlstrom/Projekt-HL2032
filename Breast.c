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

/* Create a package struct to transmit (transmit and recive data from gyro/acc, vectors) */
struct Package {
    float aX,aY,aZ,gX,gY,gZ;
};

////////////////////////// Define functions /////////////////////////////////////////

void Recive_IMU_Data(struct Package *package);

void SendToSD(float data[]);

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

    /////////////////////////////////// SD-Card variables ///////////////////////////////////////// 
    
    /* Create array for transmiting data to SD-Card & Clock to see when we should send it */
    float data[128] = {11.11,22.22,11.11,33.33,11.11,44.44,11.11,55.55,11.11,66.66};
    int clock = 0;
	
    
    
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
        Recive_IMU_Data(&package2);

        /////////////////////////////////////////////////// Math //////////////////////////////////////////////////////////// 
        Ax = package2.aX - package.aX;
        Ay = package2.aY - package.aY;
        Gx = package2.gX - package.gX;
        Gy = package2.gY - package.gY;

        PosX += Ax*Gx;
        PosY += Ay*Gy;
        
        /* Skriv kod för att navigera i ett 2D rutnät */

        /////////////////////////////////////////////////// Haptic-Feedback //////////////////////////////////////////////////////////// 


        /////////////////////////////////////////////////// SD-Card //////////////////////////////////////////////////////////// 
        data[clock] = PosX;
        data[clock + 1] = PosY;   
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
