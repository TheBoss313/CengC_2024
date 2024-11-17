/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep

*/

#include "ServoC.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
ServoC myservo;  // create servo object to control a servo

#define servopin 7  // assign pin 5 for servo
#define RED 1
#define GREEN 2
#define YELLOW 3
#define BLUE 4
#define BLACK 5
#define WHITE 6
int pos = 0;    // variable to store the servo position
// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// our RGB -> eye-recognized gamma color
byte gammatable[256];

int loop_count = 0;
int prev_color = RED;

void setup() {
    myservo.attach(servopin);  // attaches the servo on pin 5 to the servo object
    Serial.begin(9600);
    //Serial.println("Color View Test!");

    if (tcs.begin()) {
        //Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }

    // use these three pins to drive an LED
    #if defined(ARDUINO_ARCH_ESP32)
        ledcAttach(redpin, 12000, 10);
        ledcAttach(greenpin, 12000, 10);
        ledcAttach(bluepin, 12000, 10);
    #else
        pinMode(redpin, OUTPUT);
        pinMode(greenpin, OUTPUT);
        pinMode(bluepin, OUTPUT);
    #endif

    // thanks PhilB for this gamma table!
    // it helps convert RGB colors to what humans see
    for (int i=0; i<256; i++) {
        float x = i;
        x /= 255;
        x = pow(x, 2.5);
        x *= 255;

        if (commonAnode) {
        gammatable[i] = 255 - x;
        } else {
        gammatable[i] = x;
        }
        //Serial.println(gammatable[i]);
    }
    //Setup motor A
    pinMode(12, OUTPUT); //Initiates Motor Channel A pin
    pinMode(9, OUTPUT); //Initiates Brake Channel A pin

    //Setup motor B
    pinMode(13, OUTPUT); //Initiates Motor Channel A pin
    pinMode(8, OUTPUT);  //Initiates Brake Channel A pin
}

int colour (float red, float green, float blue){
    String colourStr;
    int final= 0;
    Serial.print("R:\t"); Serial.print(int(red)); 
    Serial.print("\tG:\t"); Serial.print(int(green)); 
    Serial.print("\tB:\t"); Serial.print(int(blue));
    if (int(red) >= 90 && int(green) < 75){
        colourStr = " red!";
        final = RED;
    }
    if (int(red) < 90 && int (blue) >= 90){
        colourStr = " blue!";
        final = BLUE;
    }
    if (int(green) >=90 && int(red) < 90){
        colourStr = " green!";
        final = GREEN;
    }
    if (int(red) >= 90 && int(green) >= 90){
        colourStr = " yellow!";
        final = YELLOW;
    }
    if (int(red) < 95 && int(green) < 90 && int(blue) < 90){
        colourStr = " white!";
        final = WHITE;
    }
    if (int(red) < 120 && int(red) >= 95 && int(green) < 90 && int(blue) < 90){
        colourStr = " black!";
        final = BLACK;
    }
    Serial.print(colourStr);
return final;
}

// The commented out code in loop is example of getRawData with clear value.
// Processing example colorview.pde can work with this kind of data too, but It requires manual conversion to 
// [0-255] RGB value. You can still uncomments parts of colorview.pde and play with clear value.
void loop() {
    float red, green, blue;
    //Motor A forward 
    digitalWrite(12, HIGH); //Establishes forward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 200);   //Spins the motor on Channel A at full speed

    //Motor B forward 
    digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    analogWrite(11, 200);    //Spins the motor on Channel B at half speed
    // Initial loop to get to Yellow/Blue
    if (loop_count = 0){
        bool blu_turn = false;
        bool blu_turn_start = false;
        while true{
            for (pos = 30; pos <= 150; pos += 20){ // goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                myservo.write(pos);              // tell servo to go to position in variable 'pos'
                delay(10);

                tcs.setInterrupt(false);  // turn on LED

                delay(20);  // takes 20ms to read

                tcs.getRGB(&red, &green, &blue);

                tcs.setInterrupt(true);  // turn off LED
                int current_color = colour(red, green, blue);
                Serial.print("\n");
                if (current_color == RED) { // Blue detected
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                } else if(current_color == YELLOW && 100<pos<150){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                    prev_color = YELLOW;
                } else if(current_color == YELLOW){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                } else if(current_color==BLUE && blu_turn == false && 30 < pos < 60){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                    blu_turn_start = true;
                } else if(current_color==BLUE && blu_turn_start == false && 80 < pos < 100){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                }
                else if(current_color == BLUE && blue_turn_start == true && 30 < pos < 90){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                }else if(current_color == BLUE && blue_turn_start == true && 80<pos < 100){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                }
                else if(current_color==BLACK && blu_turn == true){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 0;  // Left motor speed
                    int speedRight = 0; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                    break;
                }


                #if defined(ARDUINO_ARCH_ESP32)
                    ledcWrite(1, gammatable[(int)red]);
                    ledcWrite(2, gammatable[(int)green]);
                    ledcWrite(3, gammatable[(int)blue]);
                #else
                    analogWrite(redpin, gammatable[(int)red]);
                    analogWrite(greenpin, gammatable[(int)green]);
                    analogWrite(bluepin, gammatable[(int)blue]);
                #endif
            }
            for (pos = 150; pos <= 30; pos -= 20){ // goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                myservo.write(pos);              // tell servo to go to position in variable 'pos'
                delay(10);

                tcs.setInterrupt(false);  // turn on LED

                delay(20);  // takes 20ms to read

                tcs.getRGB(&red, &green, &blue);

                tcs.setInterrupt(true);  // turn off LED
                int current_color = colour(red, green, blue);
                Serial.print("\n");
                if (current_color == prev_color) { // Blue detected
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                } else if(current_color == YELLOW && 100<pos<120){
                    // Adjust motor speeds based on servo position
                    int speedLeft = 165 - pos / 2;  // Left motor speed
                    int speedRight = 165 + pos / 2; // Right motor speed

                    speedLeft = constrain(speedLeft, 0, 255);   // Ensure speed stays within range
                    speedRight = constrain(speedRight, 0, 255);

                    analogWrite(3, speedLeft);  // Control left motor
                    analogWrite(11, speedRight); // Control right motor
                    prev_color = YELLOW;
                } else if(current_color == BLUE && prev_color == YELLOW){
                    mid_loop = true;
                    PREV_COLOR = BLUE;
                    break
                }
                #if defined(ARDUINO_ARCH_ESP32)
                    ledcWrite(1, gammatable[(int)red]);
                    ledcWrite(2, gammatable[(int)green]);
                    ledcWrite(3, gammatable[(int)blue]);
                #else
                    analogWrite(redpin, gammatable[(int)red]);
                    analogWrite(greenpin, gammatable[(int)green]);
                    analogWrite(bluepin, gammatable[(int)blue]);
                #endif
            }
        }
        loop_count = 1;
    }

}