#include <Servo.h>

Servo myservo;

// robot pins
int servo_pin = 6;
int echo = 5;
int trig = 4;
int wheel1[] = {10, 9, 8};
int wheel2[] = {11, 13, 12};

// others
int whl_count1 = 0;
int whl_count2 = 0;
int s1 = 255;   // speed for wheel1
int s2 = 255;   // speed for wheel2
int angles[] = {-90, 45, 90, 135, 180};   // write to servo
int normal_angles[] = {-90, -45, 0, 45, 90};   // in normal system
int ang = 5;   // length(angles)
float distance = 0;

// millis
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long aMillis = 0;
unsigned long bMillis = 0;
unsigned long interval = 200;
int stop = 0;
int i = 0;     // for angles

void enc1() {
    // works every interrupt
    whl_count1++;
}

void enc2() {
    // works every interrupt
    whl_count2++;
}

void run(int* mass, int sp, int n1) {
   // mass - wheel, sp - speed, n1 - direction
   analogWrite(mass[0], sp);
   digitalWrite(mass[1], n1);
   digitalWrite(mass[2], !n1);
}

void correct() {
    // if we try to run forward
    if ((whl_count1 != whl_count2) and
        (whl_count1 != 0) and
        (whl_count2 != 0)) {
          if (whl_count1 > whl_count2) {
            if (s2 < 253) {
              s2 += 3;
            }
          } else {
            if (s2 > 200) {
              s2 -= 3;
            }
          }
    }
    /* debug
    String f = "fuck";
    if (whl_count1 == 0) {
      Serial.println(f+" 1");
    }
    if (whl_count2 == 0) {
      Serial.println(f+" 2"); 
    }
    */
}

void setup() {
    Serial.begin(9600);                 // servo
    myservo.attach(servo_pin);

    pinMode(trig, OUTPUT);              // ultra
    pinMode(echo, INPUT);

    for(int i=0; i<3; i++) {            // wheels
        pinMode(wheel1[i], OUTPUT);
        pinMode(wheel2[i], OUTPUT);
    }

    attachInterrupt(1, enc1, CHANGE);   // encoders
    attachInterrupt(0, enc2, CHANGE);

    run(wheel1, 255, LOW);    //  stop now == 0
    run(wheel2, 255, LOW);
}

void loop() {
    whl_count1 = whl_count2 = 0;

    run(wheel1, s1, LOW);
    run(wheel2, s2, LOW);
    delay(interval);

    run(wheel1, 0, LOW);
    run(wheel2, 0, LOW);
    delay(interval);

    // now we ride forward, later whole room
    correct();

    for(int i=0; i<ang; i++) {
        // move ultra for all angles
        myservo.write(angles[i]);
        delay(500);

        digitalWrite(trig, HIGH);
        delay(10);
        digitalWrite(trig, LOW);
        distance = pulseIn(echo, HIGH);
        distance = distance / 58;

        // r1 and r2
        if (!i) {
          Serial.print(String(whl_count1)+" "+String(whl_count2)+" ");
        } else {
          Serial.print("0 0 "); 
        }
        Serial.println(String(normal_angles[i])+" "+String(int(distance)));
    }
}
