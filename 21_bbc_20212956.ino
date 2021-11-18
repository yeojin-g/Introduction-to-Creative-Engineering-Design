#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0


// global variables
Servo myservo;
int a = 78, b = 299; // unit: mm
float dist_ema, alpha; // unit: mm

void setup() {
    // initialize GPIO pins
    myservo.attach(PIN_SERVO); 
    myservo.writeMicroseconds(1450);
    
    // initialize serial port
    Serial.begin(57600);

    //initialize variable
    alpha = 0.2;
}

//적외선 센서 값 변환
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-10.0))-4.0) * 10.0;
  return val;}
  

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a); 

  //EMA filter
  dist_ema = alpha*dist_cali+(1-alpha)*dist_ema;
  
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_cali);
  Serial.print(",");
  Serial.print("ema:");
  Serial.print(dist_ema);
  Serial.print(",");
  Serial.println("Max:500");
  delay(20);

  //거리에 따른 레일 기울기 제어
  if(dist_cali < 255.0) {
    myservo.writeMicroseconds(1630);
    delay(20);
  }
  else if(dist_cali > 255.0){
    myservo.writeMicroseconds(1300);
    delay(20);
}
}
