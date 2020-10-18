#include <Arduino.h>
#include <Pixy.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Servo.h>

int y_limit[] = {0, 200};
int x_limit[] = {0, 340};

// This is the main Pixy object
Pixy pixy;

// This is the servo object
Servo servo;

void setup()
{
    Serial.begin(9600);
    Serial.print("Starting...\n");

    pixy.init();
}

void loop()
{
    static int i = 0;
    int j;
    uint16_t blocks;
    char buf[32];

    // grab blocks!
    blocks = pixy.getBlocks();

    // If there are detect blocks, print them!
    if (blocks)
    {
        i++;

        // do this (print) every 50 frames because printing every
        // frame would bog down the Arduino
        if (i%50==0)
        {
            int largestObject = 0;
            int areaLargestObject = 0;

            sprintf(buf, "Detected %d:\n", blocks);
            Serial.print(buf);
            for (j=0; j<blocks; j++)
            {
                // Check and find the largest object of signature 1
                if (y_limit[0] < pixy.blocks[j].y < y_limit[1] && x_limit[0] < pixy.blocks[j].x) {
                    int areaObject = pixy.blocks[j].height * pixy.blocks[j].width;
                    Serial.println(areaObject);
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
                pixy.blocks[largestObject].print();

            }

        }
    }

}
