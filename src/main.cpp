#include <Arduino.h>
#include <Pixy.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Servo.h>

// Constants
int y_limit[] = {0, 200};
int x_limit[] = {0, 340}; // Acts as the linear approximation for the ball
int servoPin = 9;
double setpoint_PID = 30; // set to a value between 0 and 100
double input_PID;
double output_PID;


double K_p = 1;
double K_i = 0;
double K_d = 0;

// This is the main Pixy object
Pixy pixy;

// This is the servo object
Servo myServo;

// This is the PID object
PID pid_controller(&input_PID, &output_PID, &setpoint_PID, K_p, K_i, K_d, DIRECT);

void setup()
{
    Serial.begin(9600);
    Serial.print("Starting...\n");


    pixy.init();
    myServo.attach(servoPin);
    pid_controller.SetMode(AUTOMATIC); // Turns the PID controller on
    pid_controller.SetSampleTime(100);
}

void loop()
{
    static int i = 0;
    int j;
    uint16_t blocks;
    // grab blocks!
    blocks = pixy.getBlocks();

    // if there are any blocks
    if (blocks)
    {
        i++;



            int largestObject = 0;
            int areaLargestObject = 0;
//
//            sprintf(buf, "Detected %d:\n", blocks);
//            Serial.print(buf);
            for (j=0; j<blocks; j++)
            {
                // Checks if the colour of the object is correct and if the position is in the correct area
                bool checkObjectPosition = (pixy.blocks[j].signature == 1) &&
                        (y_limit[0] < pixy.blocks[j].y) &&
                        (pixy.blocks[j].y < y_limit[1]) &&
                        (x_limit[0] < pixy.blocks[j].x) &&
                        (pixy.blocks[j].x < x_limit[1]);

                if (checkObjectPosition) {
                    int areaObject = pixy.blocks[j].height * pixy.blocks[j].width;
                    if (areaLargestObject >= areaObject) {
                        areaLargestObject = areaObject;
                        largestObject = j;
                    }

                }
                // Check for the position of the object

                // Transform the objects coordinates to meters and print the result

//
//                sprintf(buf, "  block %d: ", j);
//                Serial.print(buf);
//                pixy.blocks[j].print();
                if (i = 30) {pixy.blocks[largestObject].print();}
            }
            long inputValue = map(pixy.blocks[largestObject].x, x_limit[0], x_limit[1], 0, 100);
            input_PID = (double) inputValue;
            pid_controller.Compute();
            long servoSetpoint = map(output_PID, 0, 100, 10, 170);
            myServo.write(servoSetpoint);


    }

}
