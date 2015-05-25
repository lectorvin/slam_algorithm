int wh1[] = {10, 9, 8};
int wh2[] = {11, 12, 13};
int whl_count1 = 0;
int whl_count2 = 0;
int interval = 500;
int stop = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long aMillis = 0;
unsigned long bMillis = 0;

void run(int* mass, int sp, int np) {
    analogWrite(mass[0], sp);
    digitalWrite(mass[1], np);
    digitalWrite(mass[2], !np);
}

void enc1() {
    whl_count1++;
}

void enc2() {
    whl_count2++;
}

void setup() {
    Serial.begin(9600);
    
    attachInterrupt(0, enc1, CHANGE);
    attachInterrupt(1, enc2, CHANGE);

    for (int i=0;i<3;i++) {
        pinMode(wh1[i], OUTPUT);
        pinMode(wh2[i], OUTPUT);
    }

    run(wh1, 255, LOW);
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
            run(wh1, 255, LOW);
            run(wh2, 255, LOW);
        }
        Serial.println(String(whl_count1)+" "+String(whl_count2));
        whl_count1 = whl_count2 = 0;
    }
}
