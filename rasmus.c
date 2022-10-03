/*
    \file  main.c
    \brief USB CDC ACM device

    \version 2019-6-5, V1.0.0, demo for GD32VF103
*/

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

uint8_t txbuffer[] = "Janne";
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
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {
    
    /* Initialize LCD */
    Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(BLACK);
    LCD_ShowChar(40,20,'A',TRANSPARENT, RED);
    delay_1ms(100);
    eclic_global_interrupt_enable();                /* USART interrupt configuration */
    LCD_ShowChar(40,30,'B',TRANSPARENT, BLUE);
    delay_1ms(100);
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    LCD_ShowChar(40,40,'C',TRANSPARENT, YELLOW);
    delay_1ms(100);
    eclic_irq_enable(USART0_IRQn, 1, 0);
    LCD_ShowChar(40,50,'D',TRANSPARENT, RED);
    LCD_Wait_On_Queue();
    delay_1ms(100);
    gd_eval_com_init(EVAL_COM0);                    /* configure COM0 */
    LCD_ShowChar(40,60,'E',TRANSPARENT, BLUE);
    LCD_Wait_On_Queue();
    delay_1ms(100);
    usart_interrupt_enable(USART0, USART_INT_TBE);  /* enable USART TBE interrupt */
    LCD_ShowChar(50,20,'F',TRANSPARENT, YELLOW);
    LCD_Wait_On_Queue();  

    while(1) {
        LCD_Clear(BLACK);
        // if (usart_flag_get(USART0,USART_FLAG_RBNE)){ // USART0 RX?
        //   LCD_ShowChar(10,40,usart_data_receive(USART0), TRANSPARENT, GREEN);
        // }

        while(platsSkickaAtm < platsSkicka) {
            usart_data_transmit(USART0, txbuffer[platsSkickaAtm]); // USRAT0 TX!
            platsSkickaAtm++;
        }
        

        //while(txcount < tx_size);
        LCD_ShowChar(50,30,'G',TRANSPARENT, BLUE);
        LCD_Wait_On_Queue();
        //while(RESET == usart_flag_get(USART0, USART_FLAG_TC));

        usart_interrupt_enable(USART0, USART_INT_RBNE);

        /* wait until USART receive the receiver_buffer */
        //while(rxcount < rx_size);
        //if(rxcount == rx_size) {
            // LCD_Fill(20, 20, 40, 40, RED);
            // LCD_ShowChar(90,40,'B',TRANSPARENT, RED);
    
            while(platsTaEmotAtm < platsTaEmot){
                if(usart_flag_get(USART0,USART_FLAG_RBNE)) {
                LCD_ShowChar(60+s,40,txbuffer[platsTaEmotAtm],TRANSPARENT, RED);
                delay_1ms(200);
                }
                platsTaEmotAtm++;
                s=s+10;
            }
            
        //}
    };
}
