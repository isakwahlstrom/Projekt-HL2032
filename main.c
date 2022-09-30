/*
    \file  main.c
    \brief USB CDC ACM device

    \version 2019-6-5, V1.0.0, demo for GD32VF103
*/

#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "usart.h"

#define GRAPH_HEIGHT 30
#define EI 1
#define DI 0

int main(void)
{
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

    /* Initialize LCD */
    Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(BLACK);

    // Creates temporary vectors to erase previously drwan lines
    mpu6500_getAccel(&vec_temp);
    mpu6500_getGyro(&vec2_temp);
    
    // Initialize usartjanne
    u0init(EI);

    while (1)
    {
        
        //u0_TX_queue();

        

        if (usart_flag_get(USART0,USART_FLAG_RBNE)){ // USART0 RX?
           //LCD_ShowChar(10,50,usart_data_receive(USART0), OPAQUE, BLUE);
           LCD_ShowNum1(40,20,usart_data_receive(USART0),6,BLUE);
        }
        usart_data_transmit(USART0, vec.x); // USRAT0 TX!

        LCD_ShowChar(20,20,'X',TRANSPARENT, line_color);
        LCD_ShowChar(20,40,'Y',TRANSPARENT, line_color);
        LCD_ShowChar(20,60,'Z',TRANSPARENT, line_color);
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
        LCD_ShowNum1(40,40,vec.y,6,line_color);
        LCD_ShowNum1(40,60,vec.z,6,line_color);

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
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    }
}



// #include "gd32vf103.h"
// #include "drivers.h"
// #include "lcd.h"
// #include "usart.h"
// #define EI 1
// #define DI 0


// int main(void){
//     int ms=0, s=0, key, pKey=-1, c=0, idle=0;
//     int lookUpTbl[16]={1,4,7,14,2,5,8,0,3,6,9,15,10,11,12,13};
//     int dac=0, speed=-100;
//     int adcr, tmpr;
//     char digits[10][10]={"Zero ","One  ","Two  ","Three","Four ","Five ","Six  ","Seven","Eight","Nine "};
//     char msg[]="*";

//     t5omsi();                               // Initialize timer5 1kHz
//     colinit();                              // Initialize column toolbox
//     l88init();                              // Initialize 8*8 led toolbox
//     keyinit();                              // Initialize keyboard toolbox
//     Lcd_SetType(LCD_NORMAL);                // or use LCD_INVERTED!
//     Lcd_Init();
//     LCD_Clear(RED);
//     LCD_ShowStr(10, 10, "POLL VERSION", WHITE, TRANSPARENT);
//     u0init(EI);                             // Initialize USART0 toolbox

//     eclic_global_interrupt_enable();        // !!! INTERRUPT ENABLED !!!

//     while (1) {
//         idle++;                             // Manage Async events
//         LCD_WR_Queue();                     // Manage LCD com queue!
//         //u0_TX_Queue();                      // Manage U(S)ART TX Queue!

//         if (usart_flag_get(USART0,USART_FLAG_RBNE)){ // USART0 RX?
//           LCD_ShowChar(10,50,usart_data_receive(USART0), OPAQUE, WHITE);
//         }

//         if (t5expq()) {                     // Manage periodic tasks
//             l88row(colset());               // ...8*8LED and Keyboard
//             ms++;                           // ...One second heart beat
//             if (ms==1000){
//               ms=0;
//               l88mem(0,s++);
//               LCD_ShowStr(10, 30, digits[s%10], WHITE, OPAQUE);
//               //usart_data_transmit(USART0, (s%10)+'0'); // USRAT0 TX!
//               msg[0]=(s%10)+'0'; putstr(msg);
//             }
//             if ((key=keyscan())>=0) {       // ...Any key pressed?
//               if (pKey==key) c++; else {c=0; pKey=key;}
//               l88mem(1,lookUpTbl[key]+(c<<4));
//             }
//             l88mem(2,idle>>8);              // ...Performance monitor
//             l88mem(3,idle); idle=0;
//         }
//     }
// }
