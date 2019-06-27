/*Author : MD MIRAJ AREFIN  */


int threshold[12] = {700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700}; //Calibration
int motor_speed = 150 ;
int max_motor_speed = 200;
int min_motor_speed = 0;
int turn_speed = 100;


int rightMotor = 9;                                                    //Setup
int leftMotor = 8;
int rightMotorForward = 6;
int rightMotorBackward = 5;
int leftMotorForward = 4;
int leftMotorBackward = 3;


int sensorRead[11];                                                     //initialised variable
long sensorValue = 0;
int right_sensor;
int left_sensor;
int middle_sensor;
int error;
int lastError = 0;

#define Kp                      30                                     //Kp<Kd
#define Kd                      60


void setup()
{

  Serial.begin(9600);

  pinMode(leftMotor, OUTPUT);                                          //setup motor output pin
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);

}

void loop() { sensorReading();

  Line();
  condition();

}


void sensorReading()                                                   //Analog to digital conversion
{
  for (int i = 0; i <= 8; i++)
  {
    if (analogRead(i) < threshold[i]) sensorRead[i] = 1;
    else sensorRead[i] = 0;
    //Serial.print(sensorRead[i]);
    //Serial.print(" ");
    Serial.print(analogRead(i));
    Serial.print("   ");
  }
  sensorValue = sensorSet();
//  Serial.print(sensorValue);
  Serial.println();
}



long int sensorSet()                                                  // Converting sensorReading into one charecter
{
  long int a = 1000000000;
  long int b = 10;
  a = a + sensorRead[0];
  for (int i = 0; i < 8; i++)
  {
    a = a + (b * sensorRead[i + 1]);
    b = b * 10;
  }
  return a;
}

void error_calc()                                                   //Calculating error value
{


  switch (sensorValue)
  {

    case 1000000001:
      error = -8;
      break;
    case 1000000011:
      error = -7;
      break;
    case 1000000111:
      error = -6;
      break;
    case 1000000110:
      error = -5;
      break;
    case 1000001110:
      error = -4;
      break;
    case 1000001100:
      error = -3;
      break;
    case 1000011100:
      error = -2;
      break;
    case 1000011000:
      error = -1;
      break;
    case 1000111000:
      error = 0;
      break;
    case 1000110000:
      error = 1;
      break;
    case 1001110000:
      error = 2;
      break;
    case 1001100000:
      error = 3;
      break;
    case 1011100000:
      error = 4;
      break;
    case 1011000000:
      error = 5;
      break;
    case 1111000000:
      error = 6;
      break;
    case 1110000000:
      error = 7;
      break;
    case 1100000000:
      error = 8;

      break;
  }

}

void Line()                                                             //Set motor speed using PID algorithm
{
  error_calc();
  //Serial.println(error);
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int left_motor_speed = motor_speed - motorSpeed;
  int right_motor_speed = motor_speed + motorSpeed;

  if (left_motor_speed > max_motor_speed) left_motor_speed = max_motor_speed;
  if (left_motor_speed < min_motor_speed) left_motor_speed = min_motor_speed;
  if (right_motor_speed > max_motor_speed) right_motor_speed = max_motor_speed;
  if (right_motor_speed < min_motor_speed) right_motor_speed = min_motor_speed;

  //Serial.print("left:");
  //Serial.print(left_motor_speed);
  //Serial.println();
  //Serial.print("right:");
  // Serial.print(right_motor_speed);
  analogWrite(rightMotor, right_motor_speed);
  analogWrite(leftMotor, left_motor_speed);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
}



void Stop()
{
  analogWrite(rightMotor, motor_speed);
  analogWrite(leftMotor, motor_speed);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
}

void forward()
{

  analogWrite(rightMotor, turn_speed);
  analogWrite(leftMotor, turn_speed);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
}

void litleForward()
{
  forward();
  delay(180);
  Stop();
  delay(250);
  sensorReading();
}

void litleForwardBlack()
{
  forward();
  delay(110);
  sensorReading();
}

void turnLeft()
{

  analogWrite(rightMotor, turn_speed);
  analogWrite(leftMotor, turn_speed);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);

}

void turnRight()
{
  analogWrite(rightMotor, turn_speed);
  analogWrite(leftMotor, turn_speed);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
}


void left_till_straight()
{

  turnLeft();
  delay(300);
  sensorReading();
  while (sensorRead[4] == 0)
  {
    turnLeft();
    sensorReading();
  }

}

void right_till_straight()
{

  turnRight();
  delay(300);
  sensorReading();
  while (sensorRead[4] == 0)
  {
    turnRight();
    sensorReading();
  }

}




void condition()                                                                    //applying condition to the robot
{
  //Serial.print(sensorRead[8]);
  //Serial.print(sensorRead[4]);
  //Serial.print(sensorRead[0]);
  //Serial.println();


  if ((sensorRead[8] == 0) && (sensorRead[6] == 0) && (sensorRead[4] == 1) && (sensorRead[2] == 1) && (sensorRead[0] == 1))
  {
    litleForward();
    sensorReading();
    if ((sensorRead[8] == 0) && (sensorRead[4] == 0) && (sensorRead[0] == 0))
    {
      right_till_straight();
    }
    else if ((sensorRead[8] == 0) && (sensorRead[4] == 1) && (sensorRead[0] == 0))
    {
      right_till_straight();
    }
    else if (((sensorRead[8] == 1) || (sensorRead[7] == 1)) && (sensorRead[4] == 0) && (sensorRead[0] == 0))
    {
      //      turnLeft();
      left_till_straight();
    }

  }

  else if ((sensorRead[8] == 1) && (sensorRead[6] == 1) && (sensorRead[4] == 1) && (sensorRead[2] == 0) && (sensorRead[0] == 0))
  {
    litleForward();
    sensorReading();
    if ((sensorRead[8] == 0) && (sensorRead[4] == 0) && (sensorRead[0] == 0))
    {
      left_till_straight();
    }
    else if ((sensorRead[8] == 0) && (sensorRead[4] == 1) && (sensorRead[0] == 0))
    {
      left_till_straight();
    }
    else if ((sensorRead[8] == 0) && (sensorRead[4] == 0) && ((sensorRead[1] == 1) || (sensorRead[0] == 1)))
    {
      // turnRight();
      right_till_straight();
    }
  }

  else if ((sensorRead[8] == 0) && (sensorRead[6] == 0) && (sensorRead[4] == 0) && (sensorRead[2] == 0) && (sensorRead[0] == 1))      //only right
  {
    //turnRight();
    right_till_straight();
  }

  else if ((sensorRead[8] == 1) &&  (sensorRead[6] == 0) && (sensorRead[4] == 0) &&  (sensorRead[2] == 0) && (sensorRead[0] == 0))      //only left
  {
    // turnLeft();
    left_till_straight();
  }



  else if ((sensorRead[8] == 1) && (sensorRead[6] == 1) && (sensorRead[4] == 1) && (sensorRead[2] == 1) && (sensorRead[0] == 1))     //cross or T
  {
    litleForward();
    sensorReading();

    if (sensorRead[4] == 0)
    {

      right_till_straight();
    }

    else if (sensorRead[4] == 1)
    {

      litleForward();
      Line();
    }
    else if (sensorValue == 1111111111)
    {

      Stop();
      delay(10000);
    }
  }

}




