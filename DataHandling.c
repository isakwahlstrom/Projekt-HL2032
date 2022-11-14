#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "math.h"

// TODO: 
// Improve / Adapt the statemachine to work with the hardware.
// Fine tuning from testing.
// Maybe change the reading frequensy in the MPU driver.

// Wake-on-motion interrupt for low power operation of applications processor (from Datasheet)

void sqRoot(float nr, float *root);
int power(int nr, int n);

int main()
{
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
    LCD_Clear(1);

    /* The related data structure for the IMU, contains a vector of x, y, z floats */
    mpu_vector_t Acc;

    /* Unsigned ints and a float to get the timer value and calulate the passed time */ 
    uint64_t start_mtime, delta_mtime;
    float time = 0;

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

    /* Create the constant for 1G and one for the acctual 1G */
    float acctualG = 9.9;
    float G = 9.806;

    /* Acceleration sum and limits to detect motions */
    float totalAcc = 0;
    float movmentLimit = 0.15 * G;

    /* Floats containing the acceleration on each axis */
    float aX, aY, aZ;
    /* Ints to indecate direction along the axis */
    int negativeX = 0, negativeY = 0, negativeZ = 0;
    /* Floats to contain the delta acceleration on each axis (sum of acc, devided by times) */
    float dAX = 0, dAY = 0, dAZ = 0;
    /* Int to know how many times we're "stuck" in the movement state */
    int times = 0;
    /* The total calulated distance we've traveled */
    float distance = 0;

    /* Navigation in the room */
    float position[3] = {200.0, 200.0, 200.0};
    int posX, posY, posZ;
    int room[200][200][200];

    while (1)
    {
        // Get accelleration data (Note: Blocking read) puts a force vector with 1G = 16 384 into x, y, z directions respectively
        mpu6500_getAccel(&Acc);

        // Scale the floats of the x, y z direction indicators (Values in G-force)
        aX = Acc.x / 16384;
        aY = Acc.y / 16384;
        aZ = Acc.z / 16384;

        // Check if the values are negative, convert negative values
        if (aX < 0)
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

        // Get the total Acc vector, when still it is the gravitational pull from the earth (Seems to be a bit off??)
        totalAcc = (aX * aX) + (aY * aY) + (aZ * aZ); // 3D - Pythagoras
        sqRoot(totalAcc, &totalAcc);                  // Should add upp to ca: 1.0 G
        totalAcc = totalAcc * G;                      // Should add upp to ca: 9.8 m/s^2

        // Display the total Acc value
        LCD_ShowNum1(20, 50, totalAcc, 4, BLUE);

        // Convert the values from G to m/s^2
        aX = aX * G;
        aY = aY * G;
        aZ = aZ * G;

        // Change to the nextState
        State = nextState;

        switch (State)
        {
        case Standby:
            // Movement detected, change state to Movement
            if (totalAcc > (acctualG + movmentLimit))
            {
                nextState = Movement;
            } else nextState = Standby;
            // tmp = get_timer_value();
            // Do nothing?
            // Reset values?
            break;
        case Movement:
            // Movement done, change state to Calulation
            if ((totalAcc < (acctualG + movmentLimit)))
            {
                nextState = Calculation;
            } else nextState = Movement;

            // Get the active time and sum of acceleration from the movment
            // Get delta time from this movement.
            // Start taking time?
            if ((dAX == 0) && (dAY == 0) && (dAZ == 0)) start_mtime = get_timer_value();
            times++;
            dAX += aX;
            dAY += aY;
            dAZ += aZ;

            // we're reading values to fast, maybe we should change the frequency in the MPU driver?
            break;
        case Calculation:
            // Calculate final position when the movement is "finished".
            // Stop taking time and set "time" to the calculated delta time
            delta_mtime = get_timer_value() - start_mtime;
            time = delta_mtime / (SystemCoreClock / 4000.0);
            
            // Show the timer in ms
            LCD_ShowNum(10, 30, time, 4, YELLOW);
            time = time / 1000;
            if (dAX > 0)
            {
                dAX = dAX / times;            // The average acceleration
                dAX = dAX - aX;               // Take away the gravitational pull
                distance = dAX * time * time; // Calculate the distance (seems like it is already in cm not m?)
                if (negativeX == 0)
                    position[0] = 200 + distance * 100;
                else
                    position[0] = 200 - distance * 100;
                // LCD_ShowNum1(100, 10, dAX, 6, GREEN);
                // LCD_ShowNum1(100, 30, negativeX, 5, GREEN);
                // LCD_ShowNum1(100, 50, distance*100, 6, GREEN);
                distance = 0;
                dAX = 0;
                negativeX = 0;
            }

            if (dAY > 0)
            {
                dAY = dAY / times;            // The average acceleration
                dAY = dAY - aY;               // Take away the gravitational pull
                distance = dAY * time * time; // Calculate the distance (seems like it is already in cm not m?)
                if (negativeY == 0)
                    position[1] = 200 + distance * 100;
                else
                    position[1] = 200 - distance * 100;
                // LCD_ShowNum1(100, 10, dAY, 6, GREEN);
                // LCD_ShowNum1(100, 30, negativeY, 5, GREEN);
                // LCD_ShowNum1(100, 50, distance*100, 6, GREEN);
                distance = 0;
                dAY = 0;
                negativeY = 0;
            }

            if (dAZ > 0)
            {
                dAZ = dAZ / times;            // The avrage acceleration
                dAZ = dAZ - aZ;               // Take away the gravitational pull
                distance = dAZ * time * time; // Calculate the distance (seems like it is already in cm not m?)
                if (negativeZ == 0)
                    position[2] = 200 + distance * 100;
                else
                    position[2] = 200 - distance * 100;
                // LCD_ShowNum1(100, 10, dAZ, 6, GREEN);
                // LCD_ShowNum1(100, 30, negativeZ, 5, GREEN);
                // LCD_ShowNum1(100, 50, distance*100, 6, GREEN);
                distance = 0;
                dAZ = 0;
                negativeZ = 0;
            }
            // Calculation done, change State to Standby
            nextState = Standby;
            times = 0;

            break;
        default:
            break;
        }

        // Show the new position
        LCD_ShowNum1(100, 10, position[0], 5, GREEN);
        LCD_ShowNum1(100, 30, position[1], 5, GREEN);
        LCD_ShowNum1(100, 50, position[2], 5, GREEN);
    }
}

/* Function to get the squareroot of a number nr */
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

/* Function to get the power n of a number nr */
int power(int nr, int n)
{
    int pow = 1;
    int i;
    for (i = 1; i < n; i++)
        pow = pow * nr;
    return pow;
}



// Alternative to using totalAcc is to always subtract with AccHand, thus it has approximately the same gravitational pull ??
// However I think this solution is more reliable or at least more independent.

// In the chest device we will get the values form the hand:
// in a simular fasion as above reciving posX, posY, posZ.
// or
// just reciving the Acc and making doubble calculations here.
// We then compare the two to get an acctual / isolated distanse of the hand.
// Maybe add / change the indicator for a started movement as an change in the total vector of the hand compared to the one on the chest.

/* Define zones (Green, Yellow, Red), to enable haptic-feedback */
    // char zones[200][200][200];
    // for (int x = 100; x <= 130; x++)
    // {
    //     for (int y = 70; y <= 130; y++)
    //     {
    //         for (int z = 60; z <= 100; z++)
    //         {
    //             zones[x][y][z] = 'G';
    //         }
    //     }
    // }
    // for (int x = 130; x <= 150; x++)
    // {
    //     for (int y = 130; y <= 150; y++)
    //     {
    //         for (int z = 100; z <= 120; z++)
    //         {
    //             zones[x][y][z] = 'Y';
    //         }
    //     }
    // }
    // for (int x = 130; x <= 150; x++)
    // {
    //     for (int y = 50; y <= 70; y++)
    //     {
    //         for (int z = 40; z <= 60; z++)
    //         {
    //             zones[x][y][z] = 'Y';
    //         }
    //     }
    // }
    // for (int x = 150; x <= 200; x++)
    // {
    //     for (int y = 150; y <= 200; y++)
    //     {
    //         for (int z = 120; z <= 200; z++)
    //         {
    //             zones[x][y][z] = 'R';
    //         }
    //     }
    // }
    // for (int x = 150; x <= 200; x++)
    // {
    //     for (int y = 0; y <= 50; y++)
    //     {
    //         for (int z = 0; z <= 40; z++)
    //         {
    //             zones[x][y][z] = 'R';
    //         }
    //     }
    // }

    /* Indecator on giving haptic feedback or not */
    // int giveFeedback = 0;


    // Add a repetition to a position in the room
    // posX = position[0];
    // posY = position[1];
    // posZ = position[2];
    // room[posX][posY][posZ] += 1;
