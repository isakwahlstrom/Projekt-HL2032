#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include <stdio.h>
#include "gd32vf103v_eval.h"

#define GRAPH_HEIGHT 30
#define EI 1
#define DI 0

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE            (ARRAYNUM(txbuffer) - 1)

uint8_t txbuffer[] = "Brassi";
uint8_t rxbuffer[32];
uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = 32;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0; 
int s=40;
int platsSkicka = TRANSMIT_SIZE;
int platsSkickaAtm = 0;
int platsTaEmot = TRANSMIT_SIZE;
int platsTaEmotAtm = 0;

int main(void) {
    
    /* Initialize */
    Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(BLACK);
    LCD_ShowChar(10,10,'A',TRANSPARENT, RED);
    delay_1ms(50);
    eclic_global_interrupt_enable();                /* USART interrupt configuration */
    LCD_ShowChar(20,10,'B',TRANSPARENT, BLUE);
    delay_1ms(50);
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    LCD_ShowChar(30,10,'C',TRANSPARENT, YELLOW);
    delay_1ms(50);
    eclic_irq_enable(USART0_IRQn, 1, 0);
    LCD_ShowChar(40,10,'D',TRANSPARENT, RED);
    LCD_Wait_On_Queue();
    delay_1ms(50);
    gd_eval_com_init(EVAL_COM0);                    /* configure COM0 */
    LCD_ShowChar(50,10,'E',TRANSPARENT, BLUE);
    LCD_Wait_On_Queue();
    delay_1ms(50);
    usart_interrupt_enable(USART0, USART_INT_TBE);  /* enable USART TBE interrupt */
    LCD_ShowChar(60,10,'F',TRANSPARENT, YELLOW);
    LCD_Wait_On_Queue();  

    while(1) {
        // LCD_Clear(BLACK);

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
    };
}

            // Nästa mål: Koppla upp gyro/accel och transmitta det istället. Sen är vi nära..
