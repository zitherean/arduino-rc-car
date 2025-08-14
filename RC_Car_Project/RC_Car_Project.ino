#include <IRremote.hpp>

/////////////////////////////////////////////////////////GLOBAL VARIABLES/////////////////////////////////////////////////////////////////
int currentSpeed;
int targetSpeed;
bool blocked;

/////////////////////////////////////////////////////////CONSTANTS/////////////////////////////////////////////////////////////////
/* IR REMOTE CONSTANTS*/
const int IR_RECEIVER_PIN = 8;

enum IRBUTTONS {
  POWER_BUTTON = 0x45,
  VOL_PLUS_BUTTON = 0x46,
  FUNC_STOP_BUTTON = 0x47,
  REWIND_BUTTON = 0x44,
  PAUSE_PLAY_BUTTON = 0x40,
  FAST_FORWARD_BUTTON = 0x43,
  CHANNEL_DOWN_BUTTON = 0x07,
  VOL_MINUS_BUTTON = 0x15,
  CHANNEL_UP_BUTTON = 0x09,
  ZERO_BUTTON = 0x16,
  EQ_BUTTON = 0x19,
  ST_REPT_BUTTON = 0x0D,
  ONE_BUTTON = 0x0C,
  TWO_BUTTON = 0x18,
  THREE_BUTTON = 0x5E,
  FOUR_BUTTON = 0x08,
  FIVE_BUTTON = 0x1C,
  SIX_BUTTON = 0x5A,
  SEVEN_BUTTON = 0x42,
  EIGHT_BUTTON = 0x52,
  NINE_BUTTON = 0x4A,
};

/* MOTOR DRIVER PINS */
// These pins control the direction of the wheel
const int LEFT_IN_1 = 2; 
const int LEFT_IN_2 = 4;
const int RIGHT_IN_3 = 5;   
const int RIGHT_IN_4 = 7;
// These pins control the speed of the wheel need to be ~(PWM) pins
const int LEFT_EN_A = 9;
const int RIGHT_EN_B = 10;

/* MOTOR SPEED SETTINGS */
// A percentage with no decimals i.e. values between 0-100
const int STOP_SPEED = 0;
const int DEFAULT_MOTOR_SPEED = 60;
const int HIGH_MOTOR_SPEED = 90;

/* ULTRASONIC SENSOR PINS */
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

/* ULTRASONIC SENSOR SETTINGS */
// These can be adjusted accordingly
const unsigned long MAX_ECHO_MICROSECONDS = 30000UL; // How long ultrasonic sensor takes to timeout
const int SONAR_SAMPLE_SIZE = 5; // How many times the sensor takes a sample before averaging the distance
const float STOP_DISTANCE_CENTIMETRES = 30.0;
const float CLEAR_DISTANCE_CENTIMETRES = 40.0; // This is coupled to the value above

/* OTHER CONSTANTS */
const int MAX_BYTE_VALUE = 255;  
const int MIN_BYTE_VALUE = 0;
const float SPEED_OF_SOUND_CM_PER_US = 0.0343; // 343 m/s
const int ACCELERATION_VALUE = 5; // This value can be changed as needed
const unsigned long SONAR_SAMPLE_INTERVAL_MS = 75;
const unsigned long SPEED_SAMPLE_INTERVAL_MS = 10;

/////////////////////////////////////////////////////////MAIN SETUP AND LOOP/////////////////////////////////////////////////////////////////


void setup() 
{
  Serial.begin(9600);

  // Setup for IR remote
  IrReceiver.begin(IR_RECEIVER_PIN, DISABLE_LED_FEEDBACK); // Interferes with PIN 3

  // Pins that change direction of the wheels
  pinMode(LEFT_IN_1, OUTPUT);
  pinMode(LEFT_IN_2, OUTPUT);
  pinMode(RIGHT_IN_3, OUTPUT);
  pinMode(RIGHT_IN_4, OUTPUT);

  // Pins that change speed of the wheels
  pinMode(LEFT_EN_A, OUTPUT);
  pinMode(RIGHT_EN_B, OUTPUT);

  // Pins for ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW); // To ensure that trig starts from LOW

  currentSpeed = percentageToByteConverter(STOP_SPEED); // Initial starting speed of the RC car
  targetSpeed = currentSpeed;

  blocked = false;
}

void loop() 
{
  handleIR();
  updateMotorSpeed();

  float distance = averageDistanceCentimetres();
  Serial.print("DISTANCE: ");
  Serial.println(distance);

  if (!isnan(distance)){
    if (!blocked && distance <= STOP_DISTANCE_CENTIMETRES){
      stop();
      blocked = true;
    }else if (blocked && distance >= CLEAR_DISTANCE_CENTIMETRES){
      blocked = false;
    }
  }
}

/////////////////////////////////////////////////////////HELPER FUNCTIONS/////////////////////////////////////////////////////////////////

/* IR Remote handler*/
void handleIR()
{
  if (IrReceiver.decode()){
    uint8_t command = IrReceiver.decodedIRData.command;
    if (command == 0){
      IrReceiver.resume();
      return;
    }
    Serial.println(command, HEX);
    direction(command);
    IrReceiver.resume();
  }
}

/* Keyboard input */
// Note: use chars in the switch statement  
void input()
{
  if (Serial.available() > 0) {
    char command = Serial.read(); 
    Serial.print("Received: ");
    Serial.println(command);
    direction(command);
  }
}

/* Motor and motor driver*/
void direction(uint8_t command)
{
  if (blocked){
    if (command == POWER_BUTTON){
      stop();
    } else if (command == VOL_MINUS_BUTTON){
      backward();
    }else{
      // forward/left/right not possible
      Serial.println("An obstacle is in the way. Reverse!");
      setSpeed(STOP_SPEED);
    }
    return;
  }

  switch (command){
    case PAUSE_PLAY_BUTTON:
      forward();
      break;
    case VOL_PLUS_BUTTON:
      fastForward();
      break;
    case VOL_MINUS_BUTTON:
      backward();
      break;
    case POWER_BUTTON:
      stop();
      break;
    case REWIND_BUTTON: 
      left();
      break;
    case FAST_FORWARD_BUTTON:
      right();
      break;      
    default:
      stop();
  }
}

void forward()
{
  digitalWrite(LEFT_IN_1, HIGH);
  digitalWrite(LEFT_IN_2, LOW);
  digitalWrite(RIGHT_IN_3, HIGH);
  digitalWrite(RIGHT_IN_4, LOW);

  setSpeed(DEFAULT_MOTOR_SPEED);
}

void fastForward()
{
  digitalWrite(LEFT_IN_1, HIGH);
  digitalWrite(LEFT_IN_2, LOW);
  digitalWrite(RIGHT_IN_3, HIGH);
  digitalWrite(RIGHT_IN_4, LOW);

  setSpeed(HIGH_MOTOR_SPEED);
}

void backward()
{
  digitalWrite(LEFT_IN_1, LOW);
  digitalWrite(LEFT_IN_2, HIGH);
  digitalWrite(RIGHT_IN_3, LOW);
  digitalWrite(RIGHT_IN_4, HIGH);

  setSpeed(DEFAULT_MOTOR_SPEED);
}

void stop()
{
  // first decelerate
  setSpeed(STOP_SPEED);

  // then turn off engine
  digitalWrite(LEFT_IN_1, LOW);
  digitalWrite(LEFT_IN_2, LOW);
  digitalWrite(RIGHT_IN_3, LOW);
  digitalWrite(RIGHT_IN_4, LOW);
}

void right()
{
  digitalWrite(LEFT_IN_1, HIGH);
  digitalWrite(LEFT_IN_2, LOW);
  digitalWrite(RIGHT_IN_3, LOW);
  digitalWrite(RIGHT_IN_4, HIGH);

  if (currentSpeed == 0){
    setSpeed(DEFAULT_MOTOR_SPEED);
  }
}

void left()
{
  digitalWrite(LEFT_IN_1, LOW);
  digitalWrite(LEFT_IN_2, HIGH);
  digitalWrite(RIGHT_IN_3, HIGH);
  digitalWrite(RIGHT_IN_4, LOW);

  if (currentSpeed == 0){
    setSpeed(DEFAULT_MOTOR_SPEED);
  }
}

void updateMotorSpeed()
{
  static unsigned long lastSpeedSample = 0;

  if (!shouldRunTask(lastSpeedSample, SPEED_SAMPLE_INTERVAL_MS)){
    return;
  }

  if (currentSpeed < targetSpeed){
    currentSpeed = min(currentSpeed + ACCELERATION_VALUE, targetSpeed); // clamp to target speed in case current speed goes above
  } else if (currentSpeed > targetSpeed){
    currentSpeed = max(currentSpeed - ACCELERATION_VALUE, targetSpeed);
  } else{
    return;
  }

  analogWrite(LEFT_EN_A, currentSpeed);
  analogWrite(RIGHT_EN_B, currentSpeed);
}

void setSpeed(int targetPercentage)
{
  targetPercentage = constrain(targetPercentage, 0, 100);
  targetSpeed = percentageToByteConverter(targetPercentage);
}

int percentageToByteConverter(int percentage)
{
  return MAX_BYTE_VALUE * (float(percentage) / 100.0);
}

/* Ultrasonic sensor */
float readDistanceCentimetres()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, MAX_ECHO_MICROSECONDS);

  if (duration == 0){
    return NAN;
  } 

  return (duration * SPEED_OF_SOUND_CM_PER_US) / 2; // divide by 2 - sound waves travel to the object and back 
}

float averageDistanceCentimetres()
{
  static unsigned long lastSonarSample = 0;
  static int count = 0;
  static int index = 0;
  static float samples[SONAR_SAMPLE_SIZE];
  static float sum = 0.0;
  static float lastAverage = NAN;

  if (!shouldRunTask(lastSonarSample, SONAR_SAMPLE_INTERVAL_MS)){
    return lastAverage;
  }

  float distance = readDistanceCentimetres();
  if (isnan(distance)){
    return lastAverage;
  } 

  if (count < SONAR_SAMPLE_SIZE){
    samples[index] = distance;
    sum += distance;
    index = (index + 1) % SONAR_SAMPLE_SIZE;  
    count++;
  } else{
    sum -= samples[index];
    samples[index] = distance;
    sum += distance;
    index = (index + 1) % SONAR_SAMPLE_SIZE;
  }

  lastAverage = sum / count;

  return lastAverage;
}

/* Achieves "threading" */
bool shouldRunTask(unsigned long &lastTime, const unsigned long LAST_INTERVAL_MS)
{
  unsigned long time = millis();
  if ((time - lastTime) >= LAST_INTERVAL_MS){
    lastTime = time; 
    return true;
  }
  return false;
}