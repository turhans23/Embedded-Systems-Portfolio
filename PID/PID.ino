#include "string.h"

#define CLK 2
#define DT 3
#define PWM_PIN 9 // Motorun hız kontrol pini (PWM)
#define DIR1 4    // Motorun yön kontrol pini 1
#define DIR2 5    // Motorun yön kontrol pini 2

// Potansiyometreler 
#define POT_KP A1
#define POT_KI A2
#define POT_KD A3

// Seri haberleşme değişkenleri
String buffer = "";     // Gelen veriyi anlık saklama
String maindata = "";   

// Encoder ve Hız değişkenleri
volatile long encoderCount = 0;
long prevEncoderCount = 0;
const int ENCODER_CPR = 600; 


double Kp ;   //  0.22
double Ki ;   //  0.852
double Kd ;   //  0.004

double desired_speed_rpm = 0.0; // Hedef hız 
double actualSpeed_rpm = 0.0;  // Gerçek hız 
double error = 0, lastError = 0;
double integral = 0, derivative = 0;
double output_pwm = 0;

// Zaman değişkenleri
unsigned long lastTime = 0;
unsigned long sampleTime = 100; 

// Seri veriden alınacak motor yönü 
int motor_direction_set = 0; 

// Encoder sayma fonksiyonu 
void countEncoder() {
  // DT pini yüksekse ileri, alçaksa geri sayım
  if (digitalRead(DT) == HIGH)
    encoderCount++;
  else
    encoderCount--;
}

void setup() {
  // Encoder pinleri
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLK), countEncoder, RISING);

// Motor sürücü pinleri
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  pinMode(POT_KP, INPUT);
  pinMode(POT_KI, INPUT);
  pinMode(POT_KD, INPUT);

  Serial.begin(9600);
  Serial.println("PID Motor Kontrol Baslatildi");
}

void loop() {

  // Seri Haberleşme ve Veri Ayrıştırma //
  if (Serial.available()) {
    char c = Serial.read(); 
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        maindata = buffer;
        buffer = ""; 


        if (maindata.startsWith("S") && maindata.length() >= 3) {

          motor_direction_set = maindata.substring(1, 2).toInt(); 


          int desired_speed_input = maindata.substring(2).toInt(); 
          desired_speed_rpm = map(desired_speed_input, 0, 99, 0, 400); 

          if (motor_direction_set == 1) {
            desired_speed_rpm = -abs(desired_speed_rpm);
          } else {
            desired_speed_rpm = abs(desired_speed_rpm);
          }

        }
      }
    } else {
      buffer += c; // Gelen karakteri buffer'a ekle
    }
  }

  Kp = analogRead(POT_KP) / 500.0;   
  Ki = analogRead(POT_KI) / 500.0;   
  Kd = analogRead(POT_KD) / 10000.0;   

  unsigned long now = millis();

  if (now - lastTime >= sampleTime) {
    lastTime = now;


     
    noInterrupts();
    long count = encoderCount;
    interrupts();

    long countDiff = encoderCount - prevEncoderCount;
    actualSpeed_rpm = (countDiff * 60000.0) / (ENCODER_CPR * sampleTime); 

 prevEncoderCount = encoderCount;


    error = desired_speed_rpm - actualSpeed_rpm;
    integral += error * (sampleTime / 1000.0);
    integral = constrain(integral, -200, 200); 

    // Türev terimi
    derivative = (error - lastError) / (sampleTime / 1000.0);
    
    // PID Çıkışı
    output_pwm = Kp * error + Ki * integral + Kd * derivative;

    // Çıkış sınırI
    output_pwm = constrain(output_pwm, -255, 255);
    

// Çıkışın işaretine göre motor yönü belirlenir
    if (output_pwm > 0) {
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      analogWrite(PWM_PIN, (int)output_pwm); 
    } else if (output_pwm < 0) {
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM_PIN, (int)-output_pwm); 
    } else {
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, LOW);
      analogWrite(PWM_PIN, 0); 
    }

    lastError = error;
    Serial.print("Hedef RPM: ");
    Serial.print(desired_speed_rpm);
    Serial.print(" | Gerc. RPM: ");
    Serial.print(actualSpeed_rpm);
    Serial.print(" | Cikis PWM: ");
    Serial.print(output_pwm);
    Serial.print(" | Hata: ");
    Serial.print(error);
    Serial.print(" | Kp: ");
    Serial.print(Kp, 2);
    Serial.print(" | Ki: ");
    Serial.print(Ki, 3);
    Serial.print(" | Kd: ");
    Serial.println(Kd, 3);
 }
}


