/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Servo.h>

#define BAUD_RATE 115200
#define HORIZONTAL_MOTOR_PIN 9
#define VERTICAL_MOTOR_PIN 10
#define VERTICAL_ANGLE_UP 0  // 0: all the way up, 180: all the way down

Servo horizontalMotor;
Servo verticalMotor;

void setup() {
    Serial.begin(BAUD_RATE);
    horizontalMotor.attach(HORIZONTAL_MOTOR_PIN);  // motor to move the fingers in the xy plane
    verticalMotor.attach(VERTICAL_MOTOR_PIN);  // motor to move the fixed finger in the z plane
}

void loop() {
    if (Serial.available() > 0) {
        // read the incoming byte. The MSB is the direction, the 7 LSBs is the angle
        // we know that the angle must be between 0 and 89 degrees so 7 bits is enough
        // we know that the MSB is either 1 (down) or 0 (up)
        byte data = Serial.read();
        int direction = (data >> 7) & 0x01;  // MSB: 1 or 0
        int angle = data & 0x7F;  // LSBs: 0 to 127

        // move the planar motor first
        horizontalMotor.write(angle);

        // move the vertical finger motor
        if (direction == 1) {
            verticalMotor.write(180);  // down
        } else {
            verticalMotor.write(VERTICAL_ANGLE_UP);  // up
        }
    }
}