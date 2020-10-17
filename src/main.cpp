#include <Arduino.h>
#include <pixy.h>

//void setup() {
//// write your initialization code here
//pinMode(13, OUTPUT);
//}
//
//void loop() {
//// write your code here
//digitalWrite(13, HIGH);
//delay(1000);
//digitalWrite(13, LOW);
//delay(1000);
//}


#include <SPI.h>
#include <Pixy.h>

// This is the main Pixy object
Pixy pixy;

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
            sprintf(buf, "Detected %d:\n", blocks);
            Serial.print(buf);
            for (j=0; j<blocks; j++)
            {
                // Check and find the largest object of signature 1

                // Check for the position of the object

                // Transform the objects coordinates to meters and print the result


                sprintf(buf, "  block %d: ", j);
                Serial.print(buf);
                pixy.blocks[j].print();
            }
        }
    }

}
