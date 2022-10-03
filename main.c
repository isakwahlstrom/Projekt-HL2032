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

uint8_t txbuffer[] = "\n\rUSART interrupt test\n\r";
uint8_t rxbuffer[32];
uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = 32;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0; 

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {
    
    /* Initialize LCD */
    Lcd_SetType(LCD_NORMAL);
    Lcd_Init();
    LCD_Clear(BLACK);

    // Initialize usartjanne
    // u0init(EI);

	/* USART interrupt configuration */
    eclic_global_interrupt_enable();
	LCD_ShowChar(40,20,'A',TRANSPARENT, GREEN);
	delay_1ms(100);
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
	LCD_ShowChar(40,40,'B',TRANSPARENT, GREEN);
	delay_1ms(100);
    eclic_irq_enable(USART0_IRQn, 1, 0);
	LCD_ShowChar(40,60,'C',TRANSPARENT, GREEN);
	delay_1ms(100);
		
    /* configure COM0 */
    gd_eval_com_init(EVAL_COM0);
	LCD_ShowChar(80,20,'D',TRANSPARENT, GREEN);
	delay_1ms(100);
    /* enable USART TBE interrupt */  
    usart_interrupt_enable(USART0, USART_INT_TBE);
	LCD_ShowChar(80,40,'E',TRANSPARENT, GREEN);
	delay_1ms(100);

	usart_interrupt_enable(USART0, USART_INT_RBNE);
	LCD_ShowChar(80,60,'F',TRANSPARENT, GREEN);
	delay_1ms(100);
    
	while(1) {
        /* wait until USART send the transmitter_buffer */
        while(txcount < tx_size);
			LCD_ShowChar(120,20,'G',TRANSPARENT, BLUE);
			LCD_Wait_On_Queue();

        while(RESET == usart_flag_get(USART0, USART_FLAG_TC));

		LCD_ShowChar(120,40,'H',TRANSPARENT, YELLOW);
        
		/* wait until USART receive the receiver_buffer */
        while(rxcount < rx_size);
        if(rxcount == rx_size) {
            if(usart_flag_get(USART0,USART_FLAG_RBNE)) {
                LCD_ShowChar(120,60,'I',TRANSPARENT, RED);
				LCD_Wait_On_Queue();
            }
        }
		
    };
}

// /*!
//     \file  main.c
//     \brief USART transmit and receive interrupt

//     \version 2019-6-5, V1.0.0, firmware for GD32VF103
// */

// #include "gd32vf103.h"
// #include <stdio.h>
// #include "gd32vf103v_eval.h"

// #define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
// #define TRANSMIT_SIZE            (ARRAYNUM(txbuffer) - 1)



