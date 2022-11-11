#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "math.h"

// It seems to work but some values are a bit off and we're still getting a lot of disturbance.

// To fix the disturbance we'll:
// DONE! create an addapted constant for the initial gravitational pull to compare with the sum value.
// add a exakt timer from the internal System Clock.
// DONE! adjust the calculations of the total acceleration on the IMU
// adjust the movementLimit.
// fine tune from testing.

// Maybe change the reading frequensy in the MPU driver.
// Maybe add / change the indicator for a started movement as an change in the total vector of the hand compared to the one on the chest.

// Wake-on-motion interrupt for low power operation of applications processor ??? From Datasheet

void sqRoot(float nr, float *root);
int power(int nr,int n);

int main(){
	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    // We're just dependent on knowing the Acceleration for this solution. Should we add Gyro, for what purpose?
    // 2 of them might be needed depending on how we receive  information from the arm.
    mpu_vector_t Acc, AccHand;

    // 
    uint64_t start_mtime, final_mtime;

    

    enum statemachine{standby, movement, calculation};
    enum statemachine State, nextState;
    State = standby;
    nextState = standby;

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
    
    // Create the constant G and a timer (should be changed to a reliable / correct clock)
    float acctualG = 9.9;
    float G = 9.806;
    float time = 0;

    // Acceleration sum and limits to detect motions.
    float totalAcc = 0;
    float movmentLimit = 0.5*G;

    // Navigation indicators, local help variables.
    float aX, aY, aZ;
    int negativeX = 0, negativeY = 0, negativeZ = 0;
    float dAX = 0, dAY = 0, dAZ = 0;
    float distance = 0;

    // Navigation in the room
    float position[3] = {200.0,200.0,200.0};
    int room[200][200][200];
    int posX, posY, posZ;

    // Define zones, to enable haptic-feedback
    char zones[200][200][200];
    for (int x = 0; x < 30; x++){
      for (int y = 0; y < 30; y++){
            for (int z = 0; z < 30; z++){
                  zones[x][y][z] = 'G';
            }
      }
    }

    int green[3][2] = {100,100,100,
                        100,100,100};
    int yellow[3][2] = {100,100,100,
                        100,100,100};
    int red[3][2] = {100,100,100,
                    100,100,100};

    int giveFeedback = 0;

    while(1){
        State = nextState;
        // Get accelleration data (Note: Blocking read) puts a force vector with 1G = 16 384 into x, y, z directions respectively
        mpu6500_getAccel(&Acc);

        // Scale the floats of the x, y z direction indicators (Values in G-force)
        aX = Acc.x / 16384;
        aY = Acc.y / 16384;
        aZ = Acc.z / 16384;

        // Check if the values are negative, convert negative values
        if(aX < 0) {
            aX = aX*(-1);
            negativeX = 1;
        }
        if(aY < 0) {
            aY = aY*(-1);
            negativeY = 1;
        }
        if(aZ < 0) {
            aZ = aZ*(-1);
            negativeZ = 1;
        }

        // Alternative to using totalAcc is to always subtract with AccHand, thus it has approximately the same gravitational pull ??
        // However I think this solution is more reliable or at least more independent.

        // Get the total Acc vector, when still it is the gravitational pull from the earth (Seems to be a bit off??)
        totalAcc = (aX*aX) + (aY*aY) + (aZ*aZ);         // 3D - Pythagoras
        sqRoot(totalAcc,&totalAcc);                     // Should add upp to ca: 1.0 G
        totalAcc = totalAcc*G;                          // Should add upp to ca: 9.8 m/s^2

        // Display the total Acc value
		LCD_ShowNum1(20,50,totalAcc,4,BLUE);

        // Convert the values from G to m/s^2
        aX = aX*G;
        aY = aY*G;
        aZ = aZ*G;

        // Get the active time and sum of acceleration from the movment
        if(totalAcc > (acctualG + movmentLimit)){
            nextState = movement;
            if(State == standby) start_mtime = get_timer_value() + (SystemCoreClock/4000.0 *100);
            else start_mtime += (SystemCoreClock/4000.0 *100);
            dAX += aX;
            dAY += aY;
            dAZ += aZ;
            // delay_1ms(50);     // we're reading values to fast, maybe we should change the frequency in the MPU driver?
        }

        final_mtime = get_timer_value();

        if((final_mtime > start_mtime) && (State == movement)){
            nextState = calculation;
        }
        
        // Calculate final position when the movement is "finished".
        if((State == calculation) && (totalAcc < (acctualG + movmentLimit))){
            time = final_mtime - start_mtime;
            if (dAX > 0){
                dAX = dAX / time;           // The avrage acceleration 
                dAX = dAX - aX;             // Take away the gravitational pull
                distance = dAX*time*time;   // Calculate the distance (seems like it is already in cm not m?)
                if(negativeX == 0) position[0] = 200 + distance;
                else position[0] = 200 - distance;
                dAX = 0;
                negativeX = 0;
            }
            if (dAY > 0){
                dAY = dAY / time;           // The avrage acceleration 
                dAY = dAY - aY;             // Take away the gravitational pull
                distance = dAY*time*time;   // Calculate the distance (seems like it is already in cm not m?)
                if(negativeY == 0) position[1] = 200 + distance;
                else position[1] = 200 - distance;
                dAY = 0;
                negativeY = 0;
            }
            if (dAZ > 0){
                dAZ = dAZ / time;           // The avrage acceleration 
                dAZ = dAZ - aZ;             // Take away the gravitational pull
                distance = dAZ*time*time;   // Calculate the distance (seems like it is already in cm not m?)
                if(negativeZ == 0) position[2] = 200 + distance;
                else position[2] = 200 - distance;
                dAZ = 0;
                negativeZ = 0;
            }
            distance = 0;
            nextState = standby;
            // Add a repetition to a position in the room
            posX = position[0];
            posY = position[1];
            posZ = position[2];
            room[posX][posY][posZ] += 1; 
        }

        // In the cheast device we will get the values form the hand:
        // in a simular fasion as above reciving posX, posY, posZ.
        // or 
        // just reciving the Acc and making doubble calculations here.
        // We then compare the two to get an acctual / isolated distanse of the hand.
        
        // Show the timer
        LCD_ShowNum(10,30,time,4,YELLOW);
        
        // Show the new position
        LCD_ShowNum1(100,10,position[0],5,GREEN);
        LCD_ShowNum1(100,30,position[1],5,GREEN);
        LCD_ShowNum1(100,50,position[2],5,GREEN);
        delay_1ms(50);
    }

}

void sqRoot(float nr, float *root){
        int i = 1, j = 1;
        float x0 = 1.0, xn = 1.0;

        for(i=1, j=1; i < nr; i=i*10, j++)
            if(nr/i == 0)
                i=nr;
        i = i/10;
        j = j-1;
        if(j > 1) x0 = j*power(10,j/2);

        for(int a = 1; a <= 10; a++){
            xn = 0.5*(x0 + (nr/x0));
            x0 = xn;
        }

        *root = xn;
    }

int power(int nr,int n) {
    int pow = 1;
    int i;
	
    for(i = 1; i < n; i++)
        pow = pow*nr;
	
    return pow;
}
