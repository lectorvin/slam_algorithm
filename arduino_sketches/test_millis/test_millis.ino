#include <Servo.h>

Servo myservo;

// robot's pins
int servo_pin = 6;
int echo = 5;
int trig = 4;
int wh1[] = {10, 9, 8};
int wh2[] = {11, 13, 12};

// others
int whl_count1 = 0;
int whl_count2 = 0;
int s1 = 255;   // speed for wheel1
int s2 = 255;   // speed for wheel2
int angles[] = {0, -90, 45, 90, 135, 180};        // write to servo
int normal_angles[] = {0, -90, -45, 0, 45, 90};   // in normal system
int ang = 6;   // length(angles)
float distance = 0;

// millis
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long aMillis = 0;
unsigned long bMillis = 0;
unsigned long interval = 200;
unsigned long servo_interval = 500;
int stop = 0;    // bool

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
}

void setup() {
    Serial.begin(9600);                 // servo
    myservo.attach(servo_pin);

    pinMode(trig, OUTPUT);              // ultra
    pinMode(echo, INPUT);

    for(int i=0; i<3; i++) {            // wheels
        pinMode(wh1[i], OUTPUT);
        pinMode(wh2[i], OUTPUT);
    }

    attachInterrupt(1, enc1, FALLING);   // encoders
    attachInterrupt(0, enc2, FALLING);

    run(wh1, 255, LOW);    //  stop now == 0
    run(wh2, 255, LOW);
}

void loop() {
    currentMillis = millis();
    if (currentMillis-previousMillis > interval) {
        previousMillis = currentMillis;
        
        stop = !stop;
        if (stop) {
            run(wh1, 0, LOW);
            run(wh2, 0, LOW);
        } else {
            run(wh1, s1, LOW);
            run(wh2, s2, LOW);
            return;
        }

        // now we ride forward, later whole room
        correct();

        int i = 0;
        int write = 1;
        bMillis = millis() - servo_interval - 1;
        
        while (true) {
            if (i == ang) {
              break;
            }
            
            if (write) {
              if (angles[i]!=0) {
                write = 0;
                myservo.write(angles[i]);  // move ultra for all angles
              }
              i++;
            }

            aMillis = millis();
            if (aMillis-bMillis > servo_interval) {
                bMillis = aMillis;
                if (angles[i]!=0) {
                  write = 1;
  
                  digitalWrite(trig, HIGH);
                  delay(10);   // unfortunatelly, we need it
                  digitalWrite(trig, LOW);
                  distance = pulseIn(echo, HIGH);
                  distance = distance / 58;
  
                  Serial.println(String(whl_count1)+" "+String(whl_count2)+" "+
                             String(normal_angles[i])+" "+String(int(distance)));
                }

                whl_count1 = whl_count2 = 0;
            }
        }
    }
}
