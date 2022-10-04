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
// From Linus
uint8_t txbuffer[] = "Brassi";
uint8_t rxbuffer[32];
uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = 32;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0;
// Our own 
uint8_t txData[] = "prippo";
int s=40;
int platsSkicka = TRANSMIT_SIZE;
int platsSkickaAtm = 0;
int platsTaEmot = TRANSMIT_SIZE;
int platsTaEmotAtm = 0;

/* Create a package struct to transmit (transmit and recive data from gyro/acc, vectors) */

/* Main function */

int main(void) {
	////////////////////////// Initialize LCD /////////////////////////////////////////
    
	Lcd_SetType(LCD_NORMAL);
    Lcd_Init();
    LCD_Clear(BLACK);

	/* int to write T/R characters on LCD screen */
	int x = 0;
	int y = 0;
	int x2 = 80;
	int y2 = 0;

	///////////////////////// Initialize Acc & Gyro ///////////////////////////////////

	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t vec, vec_temp;
    mpu_vector_t vec2, vec2_temp;
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
    
	// u0init(EI);

	/* USART interrupt configuration */
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(USART0_IRQn, 1, 0);
		
    /* configure COM0 */
    gd_eval_com_init(EVAL_COM0);
	
    /* enable USART TBE interrupt */  
    usart_interrupt_enable(USART0, USART_INT_TBE);
	/* enable USART RBNE interrupt */  
	//usart_interrupt_enable(USART0, USART_INT_RBNE);

	/* Infinity while loop to always check gyro/acc & sedn/recive */
    
	while(1) {
		/////////////////////////////////// USART /////////////////////////////////////////
		//usart_interrupt_enable(USART0, USART_INT_TBE);
		//LCD_ShowChar(120,40,'E',TRANSPARENT, YELLOW);
		//delay_1ms(100);

        /////////////////////// Kalles ///////////////////////////////////////////
        /*Transfer
        while((txcount < tx_size) || (x<80 && y<80)){
			usart_data_transmit(USART0, txData[txcount++]);
			LCD_ShowChar(x,y,'T',TRANSPARENT, GREEN);
			delay_1ms(200);
			y+=10;
			if (y > 80) {
				y = 0;
				x += 20;
			}
		}
        if(RESET == usart_flag_get(USART0, USART_FLAG_TC)){
			delay_1ms(200);
			LCD_Clear(BLACK);
		}	
        
        // Recive
        //if(usart_flag_get(USART0,USART_FLAG_RBNE)) {
			while(rxcount < rx_size){
				rxbuffer[rxcount++] = usart_data_receive(USART0);
            	LCD_ShowChar(x2,y2,'R',TRANSPARENT, RED);
				delay_1ms(200);
				y2+=10;
				if (y2 > 80) {
				y2 = 0;
				x2 += 20;
				}
			}
        //}
        */

        ////////////////////////////////// Rasmus ////////////////////////////////////////////////////////////
        // Transfer
        while(platsSkickaAtm < platsSkicka) {                      // Skicka varje tecken i en förbestämd array
            usart_data_transmit(USART0, txbuffer[platsSkickaAtm]); // USRAT0 TX!
            platsSkickaAtm++;
        }
    
        LCD_ShowChar(10,30,'I',TRANSPARENT, BLUE);
        LCD_ShowChar(20,30,'N',TRANSPARENT, BLUE);
        LCD_ShowChar(30,30,'I',TRANSPARENT, BLUE);
        LCD_ShowChar(40,30,'T',TRANSPARENT, BLUE);
        LCD_ShowChar(55,30,'O',TRANSPARENT, BLUE);
        LCD_ShowChar(65,30,'K',TRANSPARENT, BLUE);
        LCD_Wait_On_Queue();
        //while(RESET == usart_flag_get(USART0, USART_FLAG_TC));    // Transfer complete? Ska vi använda denna?

        // Recive
        usart_interrupt_enable(USART0, USART_INT_RBNE);             // Gå över till "ta emot-läge"
    
            while(platsTaEmotAtm < platsTaEmot){
                if(usart_flag_get(USART0,USART_FLAG_RBNE)) {        // Om det finns något i RX bufferten, visa upp varje tecken
                    LCD_ShowChar(60+s,40,txbuffer[platsTaEmotAtm],TRANSPARENT, RED);        // i en array
                    delay_1ms(200);
                    LCD_Wait_On_Queue();
                }
                platsTaEmotAtm++;
                s=s+8;
            }


        ///////////////////////////////////////////////// Accel //////////////////////////////////////////////////////
		
        //LCD_ShowChar(20,20,'X',TRANSPARENT, line_color);
        //LCD_ShowChar(20,40,'Y',TRANSPARENT, line_color);
        //LCD_ShowChar(20,60,'Z',TRANSPARENT, line_color);
        //mpu6500_getGyroAccel(&vec2,&vec);
        // Calls on getAccel and getGyro at the same time

        /* Get accelleration data (Note: Blocking read) puts a force vector with 1G = 4096 into x, y, z directions respectively */
        mpu6500_getAccel(&vec);

        /* Do some fancy math to make a nice display */

        /* Green if pointing up, red if down */
        line_color = (vec.z < 0) ? RED : GREEN;
        /* Draw a unit circle (1G) */
        //Draw_Circle(40, 80 / 2, 28, BLUE);
        /* Erase last line */
        //LCD_DrawLine(40, 80 / 2, (40) + (vec_temp.y) / (4096 / 28), (80 / 2) + (vec_temp.x / (4096 / 28)), BLACK);
        /* Draw new line, scaled to the unit circle */
        //LCD_DrawLine(40, 80 / 2, (40) + (vec.y) / (4096 / 28), (80 / 2) + (vec.x / (4096 / 28)), line_color);
        /* Store the last vector in temporary so it can be erased */
        
        //LCD_ShowNum1(40,20,vec.x,6,line_color);
        //LCD_ShowNum1(40,40,vec.y,6,line_color);
        //LCD_ShowNum1(40,60,vec.z,6,line_color);

        vec_temp = vec;

        //printf("%f", vec.x);

        // LCD_DrawLine(40, 80 / 2, (40) + (vec_temp.y) / (4096 / 28), (80 / 2) + (vec_temp.x / (4096 / 28)), BLACK);
        // "Upp" --> upp, "ner" --> Ner, Vänster --> vänster, Höger --> höger
        // Positivt y = lutar mot sladden           |  Negativt y = lutar bort från sladden          (Skickar y värdet som x värde in till drawLine)
        // Positivt x = lutar bort från knappsatsen |  Negativt x = lutar mot knappsatsen            (Skickar x värdet som y värde in till drawLine)

        // Längden på linjen beror på 

        // När vi ändrar färg beroende på z. line_color = (vec2.z < 0) ? RED : GREEN; LCD_NORMAL
        // Skiftar mellan rött o grönt beroende på om den är rättvänd eller upp och ner.
        // Rött = negativt = upp och ner | Grönt = positivt = rättvänd
        
        
        /* Wait for LCD to finish drawing */
        LCD_Wait_On_Queue();
        
        ///////////////////////////////////////////////// Gyro /////////////////////////////////////////////////////

        /* Get Gyro data (Note: Blocking read???) puts a angular vector with degrees / sec into x, y, z directions respectively */
        //mpu6500_getGyro(&vec2);

        /* Do some fancy math to make a nice display */

        /* Green if pointing up, red if down */
        //line_color = (vec2.z < 0) ? RED : GREEN;
        /* Draw a unit circle (1G)  adapt to Gyro? */
        //Draw_Circle(120, 80 / 2, 28, BLUE);
        /* Erase last line */
        //LCD_DrawLine(120, 80 / 2, (120) + (-1*(vec2_temp.x)) / (2000 / 28), (80 / 2) + (vec2_temp.y / (2000 / 28)), BLACK);
        /* Draw new line, scaled to the unit circle */
        //LCD_DrawLine(120, 80 / 2, (120) + (-1*(vec2.x)) / (2000 / 28), (80 / 2) + (vec2.y / (2000 / 28)), line_color);
        /* Store the last vector so it can be erased */
        //vec2_temp = vec2;

        // LCD_DrawLine(120, 80 / 2, (120) + (vec2.x) / (2000 / 28), (80 / 2) + (vec2.y / (2000 / 28)), line_color);
        // "Upp" --> upp, "ner" --> Ner, Vänster --> höger, Höger --> vänster
        // Positivt x = lutar bort från sladden |  Negativt x = lutar mot sladden               "Rotation runt x axeln"
        // Positivt y = lutar mot knappsatsen   |  Negativt y = lutar bort från knappsatsen     "Rotation runt z axeln"

        // Längden på linjen beror på hastigheten som vinkeln ändras med.
        // Hur kan det anpassas eller tydas på ett bra sätt?

        // När vi ändrar färg beroende på z. line_color = (vec2.z < 0) ? RED : GREEN; LCD_NORMAL
        // Skiftar mellan rött o grönt efter rotationen ? Sladd mot knappsats = rött, tvärtom = grönt.
        // Grönt = moturs = positivt z | Rött = medurs = negativt z
        // "Rotation runt y axeln"

        
        /* Wait for LCD to finish drawing */
        //LCD_Wait_On_Queue();
		
    };
}
