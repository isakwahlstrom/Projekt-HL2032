#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "math.h"

// It seems to work but some values are a bit off and we're still getting a lot of disturbance.

// To fix the disturbance we'll:
// DONE! create an addapted constant for the initial gravitational pull to compare with the sum value.
// DONE! add a exakt timer from the internal System Clock.
// DONE! adjust the calculations of the total acceleration on the IMU
// DONE! adjust the movementLimit.
// fine tune from testing.

// Maybe change the reading frequensy in the MPU driver.
// Maybe add / change the indicator for a started movement as an change in the total vector of the hand compared to the one on the chest.

// Wake-on-motion interrupt for low power operation of applications processor ??? From Datasheet

void sqRoot(float nr, float *root);
int power(int nr, int n);

int main()
{
    /* The related data structure for the IMU, contains a vector of x, y, z floats*/
    // We're just dependent on knowing the Acceleration for this solution. Should we add Gyro, for what purpose?
    // 2 of them might be needed depending on how we receive  information from the arm.
    mpu_vector_t Acc, AccHand;

    //
    uint64_t start_mtime, delta_mtime, total;
    int times = 0;

    /* Create a State Machine with 3 states: Stand-by, Movement and Calculation */
    enum StateMachine
    {
        Standby,
        Movement,
        Calculation
    };
    enum StateMachine State, nextState;
    State = Standby;
    nextState = Standby;

    /* Initialize pins for I2C */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */
    mpu6500_install(I2C0);

    /* Initialize LCD */
    Lcd_SetType(LCD_NORMAL);
    Lcd_Init();
    LCD_Clear(1);

    // Create the constant G and a timer (should be changed to a reliable / correct clock)
    float acctualG = 9.9;
    float G = 9.806;
    float time = 0;

    // Acceleration sum and limits to detect motions.
    float totalAcc = 0;
    float movmentLimit = 0.05 * G;
    float velocity = 0;
    float temp = 0;

    // Navigation indicators, local help variables.
    float aX, aY, aZ;
    int negativeX = 0, negativeY = 0, negativeZ = 0;
    float dAX = 0, dAY = 0, dAZ = 0;
    float distance = 0;
    float sumAcc = 0;
    float integral = 0;

    // Navigation in the room
    float position[3] = {200.0, 200.0, 200.0};
    int room[200][200][200];
    int posX, posY, posZ;

    // Define zones, to enable haptic-feedback
    char zones[200][200][200];

   
    for (int x = 100; x <= 130; x++)
    {
        for (int y = 70; y <= 130; y++)
        {
            for (int z = 60; z <= 100; z++)
            {
                zones[x][y][z] = 'G';
            }
        }
    }
    for (int x = 130; x <= 150; x++)
    {
        for (int y = 130; y <= 150; y++)
        {
            for (int z = 100; z <= 120; z++)
            {
                zones[x][y][z] = 'Y';
            }
        }
    }
    for (int x = 130; x <= 150; x++)
    {
        for (int y = 50; y <= 70; y++)
        {
            for (int z = 40; z <= 60; z++)
            {
                zones[x][y][z] = 'Y';
            }
        }
    }
    for (int x = 150; x <= 200; x++)
    {
        for (int y = 150; y <= 200; y++)
        {
            for (int z = 120; z <= 200; z++)
            {
                zones[x][y][z] = 'R';
            }
        }
    }
    for (int x = 150; x <= 200; x++)
    {
        for (int y = 0; y <= 50; y++)
        {
            for (int z = 0; z <= 40; z++)
            {
                zones[x][y][z] = 'R';
            }
        }
    }

    int giveFeedback = 0;
    
    while (1)
    {
        if(totalAcc<17000) {
            start_mtime = get_timer_value();
            total=0;
        }
        // Get accelleration data (Note: Blocking read) puts a force vector with 1G = 16 384 into x, y, z directions respectively
        mpu6500_getAccel(&Acc);

        // Scale the floats of the x, y z direction indicators (Values in G-force)
        aX = Acc.x;
        aY = Acc.y;
        aZ = Acc.z;

        // Check if the values are negative, convert negative values
       /* if (aX < 0)
        {
            aX = aX * (-1);
            negativeX = 1;
        }
        if (aY < 0)
        {
            aY = aY * (-1);
            negativeY = 1;
        }
        if (aZ < 0)
        {
            aZ = aZ * (-1);
            negativeZ = 1;
        }
*/
        // Alternative to using totalAcc is to always subtract with AccHand, thus it has approximately the same gravitational pull ??
        // However I think this solution is more reliable or at least more independent.

        // Get the total Acc vector, when still it is the gravitational pull from the earth (Seems to be a bit off??)
        velocity = aX + aZ + aY;
        totalAcc = (aX * aX) + (aY * aY) + (aZ * aZ);   // 3D - Pythagoras
        sqRoot(totalAcc, &totalAcc);                    // Should add upp to ca: 1.0 G
        //totalAcc = totalAcc * G;                      // Should add upp to ca: 9.8 m/s^2
        // Display the total Acc value
        LCD_ShowNum1(20, 0,totalAcc, 10, RED);
        

        // Convert the values from G to m/s^2
        //aX = aX * G;
        //aY = aY * G;
        //aZ = aZ * G;
        LCD_ShowNum1(20, 50, velocity, 10, BLUE);
        
        if(totalAcc>17000) {
            delta_mtime = get_timer_value() - start_mtime;
            delta_mtime = delta_mtime/(SystemCoreClock/4000);
            total += delta_mtime;
            sumAcc += aX * delta_mtime;
            start_mtime = get_timer_value();
            //delay_1ms(1000);
        }
        

       if(totalAcc>17000 && total!=0) {
            total = total/1000;
            
            sumAcc = sumAcc/16384;
            
            //sumAcc = sumAcc -3;
            //if(temp!=0) {
            LCD_ShowNum1(20, 60, total,6,YELLOW);
            LCD_ShowNum1(20, 30, sumAcc, 6, BLUE);
            //temp = sumAcc;
            //}
           // total = 0;
            //sumAcc = 0;
        }

        /*
        Sätta in koden i Switch satsen (state machine).
        Ändra gränsen för rörelse, använda Vel istället för Acc, lägg till en bra död zon.
        Summera hastigheterna istället för Ax * time.
        Rensa upp koden.
        */

        State = nextState;

        switch (State)
        {
        case Movement:
         

            // Add a repetition to a position in the room
            posX = position[0];
            posY = position[1];
            posZ = position[2];
            room[posX][posY][posZ] += 1;
            break;
        default:
            break;
        }

        // In the chest device we will get the values form the hand:
        // in a simular fasion as above reciving posX, posY, posZ.
        // or
        // just reciving the Acc and making doubble calculations here.
        // We then compare the two to get an acctual / isolated distanse of the hand.

        // Show the new position
        LCD_ShowNum1(100, 10, temp, 5, GREEN);
        LCD_ShowNum1(100, 30, position[1], 5, GREEN);
        LCD_ShowNum1(100, 50, position[2], 5, GREEN);
        //delay_1ms(50);
    }
}

void sqRoot(float nr, float *root)
{
    int i = 1, j = 1;
    float x0 = 1.0, xn = 1.0;

    for (i = 1, j = 1; i < nr; i = i * 10, j++)
        if (nr / i == 0)
            i = nr;
    i = i / 10;
    j = j - 1;
    if (j > 1)
        x0 = j * power(10, j / 2);

    for (int a = 1; a <= 10; a++)
    {
        xn = 0.5 * (x0 + (nr / x0));
        x0 = xn;
    }

    *root = xn;
}

int power(int nr, int n)
{
    int pow = 1;
    int i;
    for (i = 1; i < n; i++)
        pow = pow * nr;
    return pow;
}

/*
    Ta in acc värden, Subtrahera med G också.
    Beräkna integral av hastighet mellan accelartion peak och pit.
    Använd deltatiden för att få tiden mellan samples.
    Rita ut en rektangel som blir större eller mindre beroende på våra rörelser, beroende av denna integral
    Integraler kan fås med addition, diskreta tidssamples.
*/
