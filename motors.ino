#include <Wire.h>

const int MPU6050_ADDR = 0x68; // MPU6050 I2C address
const int ACCEL_SCALE = 16384; // Sensitivity scale factor for accelerometer
const int GYRO_SCALE = 131;    // Sensitivity scale factor for gyroscope

// Raw sensor data
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Complementary filter constants
float alpha = 0.98; // Complementary filter coefficient for accelerometer
float dt = 0.01;    // Time interval (adjust as needed)

float roll = 0.0; // Calculated roll angle in degrees
int check=0;
float base_roll=0.0;
int update_flag=0;
boolean impact_detected = false;

unsigned long time1;
unsigned long impact_time;
unsigned long alert_delay = 8000;
//int BUZZER=3;
int BUTTON=2;
const int vibrationPin = 3;
const int BUZZER = 4;
int vib_status=0;

char data;
int enA = 10;
int in1 = 9;
int in2 = 8;
 //motor two
int enB = 5;
int in3 = 7;
int in4 = 6;
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission();
  pinMode(vibrationPin, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT);
  analogWrite(enA,200);
  analogWrite(enB,200);
}
void roll_calculate() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // Read accelerometer data
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();

  // Read gyroscope data
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
  float accelRoll = atan2(accelY, accelZ) * (180.0 / PI);
  gyroX /= GYRO_SCALE;
  roll = alpha * (roll + gyroX * dt) + (1 - alpha) * accelRoll;
}
void basevalue(){
  while(check<15){
    roll_calculate();
    base_roll+=roll;
    check++;
  }
  base_roll=base_roll/15;
  //Serial.print("base_roll:");
  //Serial.println(base_roll);
}
void alert_mpu(){
  roll_calculate();
  // Serial.print("roll");
  // Serial.println(roll);
  if((roll>(base_roll+70)) || roll<(base_roll-70)){
    if(update_flag==0){
      //Serial.println("accident alert by mpu");
      digitalWrite(BUZZER, HIGH);
      impact_detected = true;
      impact_time = millis();
      update_flag=1;
    }
    
  }
}
void vib_sensor(){
  int vibrationValue = digitalRead(vibrationPin);

  if (vibrationValue == HIGH) {
    // Vibration detected (potential accident)
    //Serial.println("Accident detected by vib!");
    
     // Delay for 1 second to avoid rapid multiple detections
     digitalWrite(BUZZER, HIGH);
     //delay(3000);
     impact_detected = true;
     impact_time = millis();
     update_flag=1;
  } 
  else{
    digitalWrite(BUZZER, LOW);
  }
}

void send_alert(){
  if(impact_detected == true){
      if(millis() - impact_time >= alert_delay) {
      digitalWrite(BUZZER, LOW);
      delay(1000);
      //send the request
      impact_detected = false;
      impact_time = 0;
      Serial.println("sended alert");
    }
  }  
}

void stop_alert(){
  if(digitalRead(BUTTON) == HIGH){
    //Serial.println("buzzer stopped");
    delay(200);
    digitalWrite(BUZZER, LOW);
    impact_detected = false;
    impact_time = 0;
    update_flag=0;
  }
}

void receive(){
  if(Serial.available()>0){
    data=Serial.read();
    switch(data){
      
      //start the car with slow speed
       case 'J'://Serial.println("slow");
      //           slow();
                 digitalWrite(BUZZER, HIGH);
               impact_detected = true;
               impact_time = millis();
                update_flag=1; 
      break;
      //high speed
      case 'I'://Serial.println("high");
                high();
      break;
      //straight
      case 'F'://Serial.println("straight");
                straight();
      break;
      //backward
      case 'B'://Serial.println("backward");
                backward();
      break;
      //right
      case 'R'://Serial.println("right");
                right();
      break;
      //left
       case 'L'://Serial.println("left");
               left();
         
      break;
      //stop
      case 'S'://Serial.println("stop");
                stop();
      break;

    }
  }
}

void slow(){
  analogWrite(enA,200);
   analogWrite(enB,200);
}
void high(){
  analogWrite(enA,250);
   analogWrite(enB,250);
}
void straight(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}
void backward(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}
void right(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}
void left(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}
void stop(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
void loop() {
  if(check<15){
    basevalue();
  }
  alert_mpu();
  //vib_sensor(); 
  send_alert();
  stop_alert();
  receive();

}
