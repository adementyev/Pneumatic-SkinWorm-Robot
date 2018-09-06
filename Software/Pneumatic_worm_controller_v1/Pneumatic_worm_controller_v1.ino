//This is for pneuBot v1 (teensy 3.6)
#define SOLENOID_1 2
#define PUMP_1     1
#define PRESSURE_1 23
#define SOLENOID_2 4
#define PUMP_2     3
#define PRESSURE_2 22
#define SOLENOID_3 6
#define PUMP_3     5
#define PRESSURE_3 21
#define SOLENOID_4 8
#define PUMP_4     7
#define PRESSURE_4 20
#define SOLENOID_5 10
#define PUMP_5     9
#define PRESSURE_5 17

#define BUTTON_1   A21
#define BUTTON_2   39
#define BUTTON_3   38
#define BUTTON_4   37 
#define LED_1       34
#define LED_2       33

const float SUPPLY_VOLTAGE = 4.99;
const float TEENSY_VOLTAGE = 3.305;
const float ADC_BITS = 8192.0;


const int PRESSURE_BUFFER_SIZE = 50; //This defines the running average filter size

float average_pressure_1, average_pressure_2, average_pressure_3, 
      average_pressure_4, average_pressure_5;
float readings_1[PRESSURE_BUFFER_SIZE], readings_2[PRESSURE_BUFFER_SIZE], 
      readings_3[PRESSURE_BUFFER_SIZE], readings_4[PRESSURE_BUFFER_SIZE],
      readings_5[PRESSURE_BUFFER_SIZE];
int readIndex_1, readIndex_2, readIndex_3, readIndex_4, readIndex_5;
float total_1, total_2, total_3,total_4,total_5;
float pressure_1, pressure_2, pressure_3, pressure_4, pressure_5;

IntervalTimer pressureTimer; 

float setPressure3, setPressure4, setPressure5; 

void setup() {
  SerialUSB.begin(0);
  analogReadResolution(13);
  
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(SOLENOID_1, OUTPUT);
  pinMode(SOLENOID_2, OUTPUT);
  pinMode(SOLENOID_3, OUTPUT);
  pinMode(SOLENOID_4, OUTPUT);
  pinMode(SOLENOID_5, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(PUMP_3, OUTPUT);
  pinMode(PUMP_4, OUTPUT);
  pinMode(PUMP_5, OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP); //0 means pushed
  pinMode(BUTTON_2, INPUT_PULLUP); //0 means pushed
  pinMode(BUTTON_3, INPUT_PULLUP); //0 means pushed
  pinMode(BUTTON_4, INPUT_PULLUP); //0 means pushed
    
  digitalWrite(LED_1, HIGH);
  testAllSolenoids(150);
  testAllPumps(300);

  pressureTimer.begin(readPressure, 10000); //0.01sec = 10 ms = 100 Hz

  

}

void readPressure(void) { 
  pressure_1 = pressureConvert_kPA_vacuum(analogRead(PRESSURE_1)); 
  pressure_2 = pressureConvert_kPA_vacuum(analogRead(PRESSURE_2)); 
  pressure_3 = pressureConvert_kPA_gauge(analogRead(PRESSURE_3)); 
  pressure_4 = pressureConvert_kPA_gauge(analogRead(PRESSURE_4)); 
  pressure_5 = pressureConvert_kPA_gauge(analogRead(PRESSURE_5)); 
  calculateAveragePressure();
}

void loop() {

digitalWrite(PUMP_1, HIGH);
digitalWrite(PUMP_2, HIGH); //closer to tether
//digitalWrite(SOLENOID_2, LOW);
digitalWrite(SOLENOID_1, HIGH);
  
  while(average_pressure_3<25) { 
        digitalWrite(PUMP_3, HIGH); 
  }
  digitalWrite(PUMP_3, LOW); 
  digitalWrite(SOLENOID_3, HIGH); 
  digitalWrite(SOLENOID_5, LOW);
  digitalWrite(SOLENOID_1, LOW);

 // digitalWrite(SOLENOID_2,HIGH);
  digitalWrite(PUMP_2, HIGH); 
  digitalWrite(PUMP_1, HIGH);
  
  while(average_pressure_5<25) { 
        digitalWrite(PUMP_5, HIGH); 
  }
  digitalWrite(PUMP_5, LOW); 
  digitalWrite(SOLENOID_5, HIGH);
  digitalWrite(SOLENOID_3, LOW);
  
  if(digitalRead(BUTTON_2)==0) {
    //setPressure4 = 30; //this is the upper one but doesn't work :( 
    setPressure5 = 30;
    setPressure3 = 30;
    delay(300);
  }
  if(digitalRead(BUTTON_3)==0) {
    setPressure4 = setPressure4-5.0;
    //digitalWrite(PUMP_2,LOW);
    //digitalWrite(PUMP_1, LOW);
    delay(300);
  }

    if(digitalRead(BUTTON_4)==0) {
   digitalWrite(PUMP_2, HIGH);
   //digitalWrite(PUMP_1, HIGH);
    delay(300);
  }

  if(average_pressure_3< setPressure3) { 
    digitalWrite(PUMP_3, HIGH); 
    //digitalWrite(SOLENOID_3, LOW); 
  }
  else{ 
    digitalWrite(PUMP_3, LOW);
  }

    if(average_pressure_4< setPressure4) { 
    digitalWrite(PUMP_4, HIGH); 
    //digitalWrite(SOLENOID_3, LOW); 
  }
  else{ 
    digitalWrite(PUMP_4, LOW);
  }

      if(average_pressure_5< setPressure5) { 
    digitalWrite(PUMP_5, HIGH); 
    //digitalWrite(SOLENOID_3, LOW); 
  }
  else{ 
    digitalWrite(PUMP_5, LOW);
  }

  // 5 makes it turn left
  /*
  if(average_pressure_5 < 25.0) { 
    digitalWrite(PUMP_5, HIGH); 
    digitalWrite(SOLENOID_5, LOW); 
  }
  else {
    digitalWrite(PUMP_5,LOW);
    digitalWrite(SOLENOID_5,HIGH);
    delay(50);
  }
  */

  /*
  if(average_pressure_4 < 25.0) { 
    digitalWrite(PUMP_4, HIGH); 
    digitalWrite(SOLENOID_4, LOW); 
    digitalWrite(PUMP_3, HIGH); 
    digitalWrite(SOLENOID_3, LOW); 
    digitalWrite(PUMP_5, HIGH); 
    digitalWrite(SOLENOID_5, LOW); 
  }
  else {
    digitalWrite(PUMP_4,LOW);
    digitalWrite(SOLENOID_4,HIGH);
    digitalWrite(PUMP_3, LOW); 
    digitalWrite(SOLENOID_3, HIGH); 
    digitalWrite(PUMP_5, LOW); 
    digitalWrite(SOLENOID_5, HIGH); 
    delay(2000);
  }
 */

  /*
  if(average_pressure_3< 25.0) { 
    digitalWrite(PUMP_3, HIGH); 
    digitalWrite(SOLENOID_3, LOW); 
  }
  else {
    digitalWrite(PUMP_3,LOW);
    digitalWrite(SOLENOID_3,HIGH);
    delay(5000);
  }
*/
  
  displayPressureData();
  delay(1);
}

void testAllSolenoids(int delayInterval) {
  digitalWrite(SOLENOID_1, HIGH); 
  delay(delayInterval); 
  digitalWrite(SOLENOID_1, LOW);
  digitalWrite(SOLENOID_2, HIGH);
  delay(delayInterval);
  digitalWrite(SOLENOID_2, LOW);
  digitalWrite(SOLENOID_3, HIGH);
  delay(delayInterval);
  digitalWrite(SOLENOID_3, LOW);
  digitalWrite(SOLENOID_4, HIGH); 
  delay(delayInterval); 
  digitalWrite(SOLENOID_4, LOW); 
  digitalWrite(SOLENOID_5, HIGH); 
  delay(delayInterval); 
  digitalWrite(SOLENOID_5, LOW);
  delay(delayInterval);
}

void testAllPumps(int delayInterval) {
  digitalWrite(PUMP_1, HIGH); 
  delay(delayInterval); 
  digitalWrite(PUMP_1, LOW);
  digitalWrite(PUMP_2, HIGH);
  delay(delayInterval);
  digitalWrite(PUMP_2, LOW);
  digitalWrite(PUMP_3, HIGH);
  delay(delayInterval);
  digitalWrite(PUMP_3, LOW);
  digitalWrite(PUMP_4, HIGH); 
  delay(delayInterval); 
  digitalWrite(PUMP_4, LOW); 
  digitalWrite(PUMP_5, HIGH); 
  delay(delayInterval); 
  digitalWrite(PUMP_5, LOW);
  delay(delayInterval);
}

float pressureConvert_kPA_vacuum(float sensorValue) { 
    // convert to voltage
    float voltage = sensorValue * (TEENSY_VOLTAGE / ADC_BITS);
    //covert to voltage in 5V range (since there is a voltage divider)
    float voltageIn5Vrange = voltage * 2.0;
    //convert voltage to kPa( equation from datasheet)
    float pressure_kPa = ((voltageIn5Vrange/SUPPLY_VOLTAGE)-0.92)/0.007652; //in KPa
    //float pressureError = 1.725*0.00762*SUPPLY_VOLTAGE;
   return pressure_kPa; 
}

float pressureConvert_kPA_gauge(float sensorValue) { 
      // convert to voltage
    float voltage = sensorValue * (TEENSY_VOLTAGE / ADC_BITS);
    //covert to voltage in 5V range (since there is a voltage divider)
    float voltageIn5Vrange = voltage * 2.0;
    //convert voltage to kPa (equation from datasheet)
    float pressure_kPa = ((voltageIn5Vrange/SUPPLY_VOLTAGE)-0.04)/0.009; //in KPa 
    //float pressureError = 1.725*0.00762*SUPPLY_VOLTAGE;
   return pressure_kPa; 
}

void displayPressureData() { 
      SerialUSB.print(average_pressure_1); //inKpa
      SerialUSB.print(" ,"); 
      SerialUSB.print(average_pressure_2); //inKpa
      SerialUSB.print(" ,"); 
      SerialUSB.print(average_pressure_3); //inKpa
      SerialUSB.print(" ,"); 
      SerialUSB.print(average_pressure_4); //inKpa      
      SerialUSB.print(" ,"); 
      SerialUSB.print(average_pressure_5); //inKpa
      SerialUSB.print(" ,"); 
      SerialUSB.println(millis());
}


void calculateAveragePressure() { 
  //Sensor 1 
  total_1 = total_1 - readings_1[readIndex_1];
  readings_1[readIndex_1] = pressure_1;
  total_1 = total_1 + readings_1[readIndex_1];
  readIndex_1 = readIndex_1 + 1;
  if (readIndex_1 >= PRESSURE_BUFFER_SIZE) {
    readIndex_1 = 0;
  }
  average_pressure_1 = (total_1 / (float) PRESSURE_BUFFER_SIZE);

  //Sensor 2
  total_2 = total_2 - readings_2[readIndex_2];
  readings_2[readIndex_2] = pressure_2;
  total_2 = total_2 + readings_2[readIndex_2];
  readIndex_2 = readIndex_2 + 1;
  if (readIndex_2 >= PRESSURE_BUFFER_SIZE) {
    readIndex_2 = 0;
  }
  average_pressure_2 = (total_2 / (float) PRESSURE_BUFFER_SIZE);

  //Sensor 3
  total_3 = total_3 - readings_3[readIndex_3];
  readings_3[readIndex_3] = pressure_3;
  total_3 = total_3 + readings_3[readIndex_3];
  readIndex_3 = readIndex_3 + 1;
  if (readIndex_3 >= PRESSURE_BUFFER_SIZE) {
    readIndex_3 = 0;
  }
  average_pressure_3 = (total_3 / (float) PRESSURE_BUFFER_SIZE);

  //Sensor 4
  total_4 = total_4 - readings_4[readIndex_4];
  readings_4[readIndex_4] = pressure_4;
  total_4 = total_4 + readings_4[readIndex_4];
  readIndex_4 = readIndex_4 + 1;
  if (readIndex_4 >= PRESSURE_BUFFER_SIZE) {
    readIndex_4 = 0;
  }
  average_pressure_4 = (total_4 / (float) PRESSURE_BUFFER_SIZE);

  //Sensor 5
  total_5 = total_5 - readings_5[readIndex_5];
  readings_5[readIndex_5] = pressure_5;
  total_5 = total_5 + readings_5[readIndex_5];
  readIndex_5 = readIndex_5 + 1;
  if (readIndex_5 >= PRESSURE_BUFFER_SIZE) {
    readIndex_5 = 0;
  }
  average_pressure_5 = (total_5 / (float) PRESSURE_BUFFER_SIZE);
  
}


  




