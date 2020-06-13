
const int PIN_LEFT_INA = 2;
const int PIN_LEFT_INB = 2;
const int PIN_LEFT_PWM = 2;

const int PIN_ENCODE_A_MOTOR_LEFT = 2;
const int PIN_ENCODE_B_MOTOR_LEFT = 3;

int pos_left = 0;
int last_pos_left = 0;

void driveLeft(float speed) //-1.0 to 1.0
{
  if (speed > 0)
  {
    digitalWrite(PIN_LEFT_INA, HIGH);
    digitalWrite(PIN_LEFT_INB, LOW);
    analogWrite(PIN_LEFT_PWM, (int)(255*speed));
  }
  else
  {
    digitalWrite(PIN_LEFT_INA, LOW);
    digitalWrite(PIN_LEFT_INB, HIGH);
    analogWrite(PIN_LEFT_PWM, (int)(-255*speed));
  }
}

void encoderLeftMotor() {
  if (digitalRead(PIN_ENCODE_A_MOTOR_LEFT) == digitalRead(PIN_ENCODE_B_MOTOR_LEFT))
  {
    pos_left++;
  }
  else {
    pos_left--;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LEFT_INA, OUTPUT);
  pinMode(PIN_LEFT_INB, OUTPUT);

  pinMode(PIN_ENCODE_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCODE_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCODE_A_MOTOR_LEFT, HIGH);               
  digitalWrite(PIN_ENCODE_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  pos_left = 0;
  Serial.begin(115200);
  Serial.println("Hello World");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pos_left);


//  w = (pos_left - last_pos_left) / 0.1; // pulse / sec
//  Serial.print(w);
//  rad_sec = w * 2 * 3.14159 / 341
//  Serial.print("\t");
//  Serial.println(rad_sec);
  
  last_pos_left = pos_left;
  delay(100);

}
