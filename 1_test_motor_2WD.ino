const int PIN_LEFT_INA = 4;
const int PIN_LEFT_INB = 5;
const int PIN_LEFT_PWM = 6;

const int PIN_RIGHT_INA = 8;
const int PIN_RIGHT_INB = 9;
const int PIN_RIGHT_PWM = 7;

const int PIN_ENCODE_A_MOTOR_LEFT = 2;
const int PIN_ENCODE_B_MOTOR_LEFT = 3;

const int PIN_ENCODE_A_MOTOR_RIGHT = 19;
const int PIN_ENCODE_B_MOTOR_RIGHT = 18;

int pos_left = 0, pos_right = 0;
int last_pos_left = 0, last_pos_right = 0;

void driveLeft(float vel) //-1.0 - 1.0
{
  if (vel > 0) {
    digitalWrite(PIN_LEFT_INA, HIGH);
    digitalWrite(PIN_LEFT_INB, LOW);
    analogWrite(PIN_LEFT_PWM, (int)(255.0 * vel));
  } else {
    digitalWrite(PIN_LEFT_INA, LOW);
    digitalWrite(PIN_LEFT_INB, HIGH);
    analogWrite(PIN_LEFT_PWM, (int)(255.0 * -vel));
  }
}

void driveRight(float vel) //-1.0 - 1.0
{
  if (vel > 0) {
    digitalWrite(PIN_RIGHT_INA, HIGH);
    digitalWrite(PIN_RIGHT_INB, LOW);
    analogWrite(PIN_RIGHT_PWM, (int)(255.0 * vel));
  } else {
    digitalWrite(PIN_RIGHT_INA, LOW);
    digitalWrite(PIN_RIGHT_INB, HIGH);
    analogWrite(PIN_RIGHT_PWM, (int)(255.0 * -vel));
  }
}

void encoderLeftMotor()
{
  if (digitalRead(PIN_ENCODE_A_MOTOR_LEFT) == digitalRead(PIN_ENCODE_B_MOTOR_LEFT))
  {
    pos_left++;
  }
  else
  {
    pos_left--;
  }
}

void encoderRightMotor()
{
  if (digitalRead(PIN_ENCODE_A_MOTOR_RIGHT) == digitalRead(PIN_ENCODE_B_MOTOR_RIGHT))
  {
    pos_right++;
  }
  else
  {
    pos_right--;
  }
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(PIN_LEFT_INA, OUTPUT);
  pinMode(PIN_LEFT_INB, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);

  pinMode(PIN_RIGHT_INA, OUTPUT);
  pinMode(PIN_RIGHT_INB, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);

  pinMode(PIN_ENCODE_A_MOTOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCODE_B_MOTOR_LEFT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODE_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  pinMode(PIN_ENCODE_A_MOTOR_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENCODE_B_MOTOR_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODE_A_MOTOR_RIGHT), encoderRightMotor, RISING);

  pos_left = 0;
  pos_right = 0;
  Serial.begin(115200);
  Serial.println("Hello World");
  driveLeft(0.4);
  driveRight(0.7);
}

void loop()
{

  //  Serial.print(pos_left);
  //  Serial.print("\t");
  //  Serial.println(pos_right);

  // find angular velocity of left motor [rad/s]
  float w_left = (pos_left - last_pos_left) / 0.1; // pulse / sec
  Serial.print(w_left);
  last_pos_left = pos_left;

  float w_right = (pos_right - last_pos_right) / 0.1;
  Serial.print("\t");
  Serial.println(w_right);
  last_pos_right = pos_right;

  //  rad_sec = w * 2 * 3.14159 / 341
  //  Serial.print("\t");
  //  Serial.println(rad_sec);


  delay(100);
}
