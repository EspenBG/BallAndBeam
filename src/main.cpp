#include <Arduino.h>
#include <Pixy.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Servo.h>

// Constants
int y_limit[] = {0, 200};
int x_limit[] = {0, 340}; // Acts as the linear approximation for the ball
int servoPin = 9;
double setPointPid = 30; // set to a value between 0 and 255
double inputPid;
double outputPid;


double Kp = 1;
double Ki = 0;
double Kd = 0;

// This is the main Pixy object
Pixy pixy;

// This is the servo object
Servo myServo;

// This is the PID object
PID pidController(&inputPid, &outputPid, &setPointPid, Kp, Ki, Kd, DIRECT);

long getPosition(uint16_t blocks);

void setup() {
    Serial.begin(9600);
    Serial.print("Starting...\n");

    pixy.init();
    myServo.attach(servoPin);
    pidController.SetMode(AUTOMATIC); // Turns the PID controller on
    pidController.SetSampleTime(100); // The delay between calculations of the PID controller
}



void loop() {
    static int i = 0;
    // Servo test comment the underlying code after servo test
    myServo.write(0);
    delay(2000);
    myServo.write(180);
    delay(2000);
    uint16_t blocks;
    // grab blocks!
    long objectPosition = pixy.getBlocks();
/* Remove line after servo test

    // Find the position of the largest object in the specified range
    if (blocks) {
        getPosition(blocks);
        i++;

        // Do we need to change the input value
        // linearizes the position to a position between 0 and 255
        long inputValue = map(objectPosition, x_limit[0], x_limit[1], 0, 255);
        inputPid = (double) inputValue;
        pidController.Compute();
        long servoSetPoint = map(outputPid, 0, 100, 10, 170);
        myServo.write(servoSetPoint);
    }
    */ //Remove line after servo test
}

/**
 * @brief Finds the center position of the largest object for the pixy blocks
 * @param blocks Give the pixy blocks to be analysed
 * @return xPosition long value for the center of the object
 */
long getPosition(uint16_t blocks) {
    int j;
    int largestObject = 0;
    int areaLargestObject = 0;

    for (j = 0; j < blocks; j++) {
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

        //if (i = 30) { pixy.blocks[largestObject].print(); }
    }
    long xPosition = pixy.blocks[largestObject].x;
    return xPosition;
}