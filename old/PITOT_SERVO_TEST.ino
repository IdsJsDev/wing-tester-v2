/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ms4525do.h"

/* 
* An MS4525DO object
*/

#include <Servo.h>
Servo myservo;

bfs::Ms4525do pres;

void setup() {
  /* Serial to display data */
  pinMode(2, OUTPUT); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  delay(300);

  // Serial.begin(9600);
  // while(!Serial){}

  Wire.begin();
  Wire.setClock(400000);
  /* 
  * I2C address of 0x28, on bus 0, with a -1 to +1 PSI range
  */
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  delay(300);
  if (!pres.Begin()) {
    // Serial.println("Error communicating with sensor");
    while(1){
      digitalWrite(LED_BUILTIN, 0);
      delay(300);
      digitalWrite(LED_BUILTIN, 1);
      delay(300);
    }
  }
  myservo.attach(9);

  int pos = 92;
  myservo.write(pos);
  delay(500);
  for (; pos <= 160; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 160; pos >= 20; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
    for (; pos <= 92; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  digitalWrite(LED_BUILTIN, 0);
  delay(300);
  digitalWrite(LED_BUILTIN, 1);
}

void loop() {
  /* Read the sensor */
  if (pres.Read()) {
    /* Display the data */
    // Serial.print(pres.pres_pa(), 6);
    // Serial.print("\t");
    // Serial.print(pres.die_temp_c(), 6);
    // Serial.print("\n");
  }
  tone(5, (uint16_t)((pres.pres_pa()*2)+6000), 100);
  delay(10);
}