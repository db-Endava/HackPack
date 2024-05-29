//20240516
// ================================================================
// ===         Serial Declarations and configurations           ===
// ================================================================
#define USB_DataP 19
#define USB_DataN 18

void INIT_Serial() {
  Serial.begin(115200);
  long Timeout = millis() + 3000;
  while (!Serial || (millis() > Timeout)) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Setup Serial Complete");
}

// ================================================================
// ===           Motor Declarations and configurations          ===
// ================================================================
// Define motor pins for each DRV8837
// Define motor pins for each DRV8837#define M1_A 4
#define M1_A_PIN 4
#define M1_B_PIN 5
#define M2_A_PIN 6
#define M2_B_PIN 7
#define M3_A_PIN 8
#define M3_B_PIN 9
#define nSLEEP_PIN 10

const int motorPins[3][2] = {
  { M1_A_PIN, M1_B_PIN },  // Motor 1: PWM_A, PWM_B
  { M2_A_PIN, M2_B_PIN },  // Motor 2: PWM_A, PWM_B
  { M3_A_PIN, M3_B_PIN }   // Motor 3: PWM_A, PWM_B
};

const int pwmChannels[6] = { 0, 1, 2, 3, 4, 5 };  // PWM channels for motors

const int Axis_Pitch = 0;
const int Axis_Launch = 1;
const int Axis_Yaw = 2;

const int STOP = 0;

// ================================================================
// ===                       IO & Motor Setup                        ===
// ================================================================
void INIT_IO() {
  // Setup LEDC for PWM
  for (int i = 0; i < 6; ++i) {
    pinMode(motorPins[i / 2][i % 2], OUTPUT);      // Set PWM pins as OUTPUT
  }
}
void INIT_Motors() {
  Serial.print("Setup Motors : ");



  pinMode(nSLEEP_PIN, OUTPUT);     // Set nSLEEP pin as OUTPUT
  digitalWrite(nSLEEP_PIN, true);  // Enable motor drivers by setting nSLEEP to true (HIGH)

  Start_Tone();  // audiable test of each motor

uint32_t PWM_frequency = 20000;
uint8_t PWM_resolution = 8;

  for (int i = 0; i < 6; ++i) {
    ledcAttachChannel(motorPins[i / 2][i % 2], PWM_frequency, PWM_resolution, pwmChannels[i]);
  }

  Serial.println("Setup Compelete");
}

// ================================================================
// ===                       Tone Startup                       ===
// ================================================================
// Tones created by each of the motors.
void Start_Tone() {
  Serial.print("Motors Tone Test : ");

  for (int i = 2; i >= 0; i--) {     //Cycle through each motor one at a time.
    long ToneTime = millis() + 200;  // sett tone time length
    bool state = 0;
    while (millis() < ToneTime) {
      digitalWrite(motorPins[i][0], state);  // set outputs state to "rotate" motor
      digitalWrite(motorPins[i][1], !state);
      long WaitTime = micros() + (100 * (i + 1));  //set time between state change
      while (micros() < WaitTime) {}               // hold state for set time
      state = !state;                              // invert state to roate opposite direction
    }
    digitalWrite(motorPins[i][0], 0);  // stop motor
    digitalWrite(motorPins[i][1], 0);
    Serial.print(" _ ");
    delay(50);  // between motor iterations
  }
  Serial.print("Test Complete : ");
}

// ================================================================
// ===                     Motor functions                      ===
// ================================================================
// motorIndex = which motor, speed = how fast in positive/negative dirrection 12 bit (-4095 to 4095)
void MotorControl(int motorIndex, int Power, int MinStartingPower = 0) {

  // Add minimum starting power to PWM output value
  int AdjustedPower = abs(Power) + MinStartingPower;

  // Map the speed value from the range 0 to 4095 to the PWM range (0 to 255)
  int pwmValue = map(AdjustedPower, 0, 4095, 0, 255);

  // Ensure the speed is within the valid range
  pwmValue = constrain(pwmValue, 0, 255);

  // Set the direction based on the sign of the speed
  if (Power > STOP) {
    // Move forward: set PWM_A to speed, PWM_B to 0
    ledcWrite(pwmChannels[motorIndex * 2], pwmValue);  // PWM_A
    ledcWrite(pwmChannels[motorIndex * 2 + 1], 0);     // PWM_B
  } else if (Power < STOP) {
    // Move backward: set PWM_A to 0, PWM_B to speed
    ledcWrite(pwmChannels[motorIndex * 2], 0);             // PWM_A
    ledcWrite(pwmChannels[motorIndex * 2 + 1], pwmValue);  // PWM_B
  } else {
    ledcWrite(pwmChannels[motorIndex * 2], STOP);      // PWM_A
    ledcWrite(pwmChannels[motorIndex * 2 + 1], STOP);  // PWM_B
  }
}

// ================================================================
// ===        LED_rgb Declarations and configurations           ===
// ================================================================
#include <Adafruit_NeoPixel.h>
#define LED_PIN 0       // Which pin on the Arduino is connected to the NeoPixels?
#define LED_COUNT 1     // How many NeoPixels are attached to the Arduino?
#define BRIGHTNESS 50  // Set BRIGHTNESS to about 1/5 (max = 255)

Adafruit_NeoPixel LED(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//Predefine common colours
const uint32_t RED = LED.Color(BRIGHTNESS, 0, 0);
const uint32_t GREEN = LED.Color(0, BRIGHTNESS, 0);
const uint32_t BLUE = LED.Color(0, 0, BRIGHTNESS);
const uint32_t YELLOW = LED.Color(BRIGHTNESS, BRIGHTNESS, 0);
const uint32_t PURPLE = LED.Color(BRIGHTNESS, 0, BRIGHTNESS);
const uint32_t CYAN = LED.Color(0, BRIGHTNESS, BRIGHTNESS);
const uint32_t WHITE = LED.Color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
const uint32_t OFF = LED.Color(0, 0, 0);

void INIT_LED_rgb() {
  Serial.print("LED Setup : ");

  LED.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  LED.show();   // Turn OFF all pixels ASAP
  LED.setBrightness(BRIGHTNESS);

  Serial.println("Setup Complete");
}


// ================================================================
// ===                  LED_rgb Set Colour Function                    ===
// ================================================================
//Choose colour and intensity (0 to 1)
void LED_SetColour(uint32_t colour) {
  LED.setPixelColor(0, colour);
  LED.show();  //  Update strip to match
}

// ================================================================
// ===                  LED_rgb Rainbow cycle function                      ===
// ================================================================
const int rainbowSpeed = 30;  // Adjust this value for the desired speed
static uint32_t lastTime = 0;
#define BRIGHTNESS 50  // Set BRIGHTNESS to about 1/5 (max = 255)

void LED_Rainbow() {
  if ((millis() - lastTime) > rainbowSpeed) {
    int pixelHue = (millis() * 2) % 65536;  // Ensure pixelHue stays within the valid range
    LED.setPixelColor(0, LED.gamma32(LED.ColorHSV(pixelHue)));
    LED.show();
    lastTime = millis();  // Save time of last movement
  }
}

// ================================================================
// ===           MPU Declarations and configurations            ===
// ================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;  // AD0 low = 0x68 (default ) or AD0 high = 0x69 - MPU6050 mpu(0x69);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

// ================================================================
// ===            MPU INTERRUPT DETECTION ROUTINE               ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
// ================================================================
// ===                        MPU SETUP                         ===
// ================================================================
#define MPU_Interrupt_Pin 1
#define SCL_Pin 2
#define SDA_Pin 3

void INIT_MPU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)

  Wire.end();  // must do this before setting pins
  Wire.setPins(SDA_Pin, SCL_Pin);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(MPU_Interrupt_Pin, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setZAccelOffset(1568);   //
  mpu.setZAccelOffset(-1795);  //
  mpu.setZAccelOffset(1806);   // 1688 factory default for my test chip
  mpu.setXGyroOffset(99);      //220
  mpu.setYGyroOffset(53);      //76
  mpu.setZGyroOffset(26);      //-85


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);


    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(MPU_Interrupt_Pin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MPU_Interrupt_Pin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


// ================================================================
// ===                        MPU LOOP                          ===
// ================================================================
float ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat gravity;     // [x, y, z]            gravity vector
Quaternion q;            // [w, x, y, z]         quaternion container
uint8_t fifoBuffer[64];  // FIFO storage buffer
double Pitch = 0, Roll = 0, Yaw = 0;

void Loop_MPU() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Pitch = ypr[1] * 180 / M_PI;
    Roll = ypr[2] * 180 / M_PI;
    Yaw = ypr[0] * 180 / M_PI;

    //Serial.println("ypr\t" + String(Yaw) + "\t" + String(Pitch) + "\t" + String(Roll));
  }
}




// ================================================================
// ===                       PID Setup                          ===
// ================================================================
#include <PID_v1.h>

double Pan_Target = 0;
double Pan_Kp = 200, Pan_Ki = 0, Pan_Kd = 0, Pan_Setpoint, Pan_Input, Pan_Output;
PID Pan_PID(&Pan_Input, &Pan_Output, &Pan_Setpoint, Pan_Kp, Pan_Ki, Pan_Kd, DIRECT);
void Pan_PID_Setup() {
  Serial.print("Pan PID Setup : ");

  Pan_PID.SetMode(AUTOMATIC);
  Pan_PID.SetOutputLimits(-4095, 4095);
  Pan_PID.SetSampleTime(1);

  Serial.println("Complete");
}

double Tilt_Target = 80;
double Tilt_Kp = 150, Tilt_Ki = 0, Tilt_Kd = 0, Tilt_Setpoint, Tilt_Input, Tilt_Output;
PID Tilt_PID(&Tilt_Input, &Tilt_Output, &Tilt_Setpoint, Tilt_Kp, Tilt_Ki, Tilt_Kd, DIRECT);
void Tilt_PID_Setup() {
  Serial.print("Tilt PID Setup : ");

  Tilt_PID.SetMode(AUTOMATIC);
  Tilt_PID.SetOutputLimits(-4095, 4095);
  Tilt_PID.SetSampleTime(1);

  Serial.println("Complete");
}

// ================================================================
// ===                       Pan Control                         ===
// ================================================================

void PanControl(int input = 0) {

  Pan_Setpoint = constrain(input, -120, 120);
  Pan_Input = Yaw;
  Pan_PID.Compute();

  if (abs(Pan_Setpoint - Pan_Input) > 1) MotorControl(Axis_Yaw, Pan_Output, 600);
  else MotorControl(Axis_Yaw, STOP);
  // Serial.println("CurrentYaw: "+String(Pan_Input) + " Target: " + String(Pan_Setpoint)+ " Output: " + String(Pan_Output));
}

void Pan_Move(double value) {
  Pan_Target += value;
  Pan_Target = constrain(Pan_Target, -100, 100);
}


// ================================================================
// ===                Home Tilt / Pitch Axis                     ===
// ================================================================

void Home_TiltAxis() {
  Serial.print("Homing Tilt/Pitch Axis : ");

  MotorControl(Axis_Pitch, -2000);  // force tilt/pitch axis to hard stop home position

  Serial.print(" _ ");
  delay(1000);
  Serial.print(" _ ");
  delay(1000);
  Serial.print(" _ ");
  delay(1000);

  MotorControl(Axis_Pitch, STOP);

  Serial.println("Complete");
}

// ================================================================
// ===                       Tilt Control                         ===
// ================================================================

void TiltControl(int input = 0) {
  int MinPower = 1500;
  Tilt_Setpoint = constrain(input, 0, 80);

  Tilt_Input = Pitch;
  Tilt_PID.Compute();

  if (abs(Tilt_Setpoint - Tilt_Input) > 1) MotorControl(Axis_Pitch, Tilt_Output, MinPower);
  else MotorControl(Axis_Pitch, STOP);
}

void Tilt_Move(double value) {
  Tilt_Target += value;
  Tilt_Target = constrain(Tilt_Target, 0, 80);
}

// ================================================================
// ===                       Launch Control                         ===
// ================================================================
// State definitions for the launch control sequence
enum LaunchState {
  READY,
  FIRE_STAGE,
  RESET_STAGE,
  LOAD_STAGE,
  COMPLETE,
  ERROR,
  START
};

// Initialize the state and timing variables
LaunchState currentLaunchState = READY;
unsigned long previousMillis = 0;
int cycleCount = 0;

// The LaunchControl function now using a non-blocking approach
void LaunchControl() {

  unsigned long currentMillis = millis();

  switch (currentLaunchState) {
    case FIRE_STAGE:
      if (currentMillis - previousMillis >= 1000) {
        MotorControl(Axis_Launch, STOP);
        if (currentMillis - previousMillis >= 1050) {
          currentLaunchState = RESET_STAGE;
          previousMillis = currentMillis;
          Serial.println("Launch Reset");
        }
      } else MotorControl(Axis_Launch, 4000);
      break;

    case RESET_STAGE:
      if (currentMillis - previousMillis >= 1000) {
        MotorControl(Axis_Launch, STOP);
        if (currentMillis - previousMillis >= 1050) {
          currentLaunchState = LOAD_STAGE;
          previousMillis = currentMillis;
          Serial.println("Launch Load");
        }
      } else MotorControl(Axis_Launch, -3500);
      break;

    case LOAD_STAGE:
      if (currentMillis - previousMillis >= 600) {
        MotorControl(Axis_Launch, STOP);
        if (currentMillis - previousMillis >= 300) {
          currentLaunchState = COMPLETE;
          previousMillis = currentMillis;
        }
      } else MotorControl(Axis_Launch, 4000);
      break;

    case COMPLETE:
      cycleCount++;
      Serial.println("Launch Complete! Cycle # :\t" + String(cycleCount));
      currentLaunchState = READY;  // Reset state to start for next cycle
      Serial.println("Launch Ready");
      break;

    case READY:
      break;

    case START:
      Serial.println("Launch Started - FIRE!");
      currentLaunchState = FIRE_STAGE;  // Reset state to start for next cycle
      previousMillis = currentMillis;
      break;

    default:
      currentLaunchState = ERROR;
  }
}

void Launch() {
  if (currentLaunchState == READY) currentLaunchState = START;
}



// ================================================================
// ===                Serial Command Functions                  ===
// ================================================================

String serialBuffer = "";  // Buffer to hold incoming data
int Position_Pan = 0;
int Position_Tilt = 80;

void Serial_USB() {
  while (Serial.available()) {               // Check if there is data on the serial port
    char inChar = (char)Serial.read();       // Read a character from the serial buffer
    Serial.print(inChar);                    // Echo the command for debugging
    if (inChar == '\n' || inChar == '\r') {  // If the end of a command is received
      if (serialBuffer.length() > 0) {
        ProcessCommand(serialBuffer);  // Process the complete command
        serialBuffer = "";             // Clear the buffer after processing
      }
    } else {
      serialBuffer += inChar;  // Add the incoming character to the buffer
    }
  }
}

void ProcessCommand(String command) {
  // Process the command string
  Serial.println("Received2 command: " + command);  // Echo the command for debugging

  // Example command processing
  if (command == "d") {
    Pan_Target += 20;  // Move pan by 10 degrees
  } else if (command == "a") {
    Pan_Target -= 20;  // Move pan by -10 degrees
  } else if (command == "w") {
    Tilt_Target += 10;  // Increase tilt by 10 degrees
  } else if (command == "s") {
    Tilt_Target -= 10;  // Decrease tilt by 10 degrees
  } else if (command == "home") {
    Home_TiltAxis();  // Home the tilt axis
    Pan_Target = 0;
  } else if (command == "l") {
    Launch();
  } else {
    Serial.println("ERROR : Unknown command");  // Handle unknown commands
  }
}

// ================================================================
// ===                    Serial Control                        ===
// ================================================================
#define UART_DataTx 21
#define UART_DataRx 20
void INIT_Serial1() {
  Serial1.begin(115200, SERIAL_8N1, UART_DataRx, UART_DataTx);
  Serial.println("Serial1 start");
}


int integerValue = 0;
bool TargetMode = 0;
void Serial_Camera() {

  if (Serial1.available()) {
    char header = Serial1.read();  // Read the header 'X'
    if (header == 'X') {
      // Read the ASCII integer as a string
      String DATA_X;
      DATA_X = Serial1.readStringUntil('\t');  // Read the data until a tab is encountered
      // Convert the ASCII integer to an int
      integerValue = atoi(DATA_X.c_str());  // Convert the String to a char array
      if (TargetMode&& (abs(integerValue)>5) )Pan_Move(constrain(int(integerValue),-3,3));

      Serial.print("Received Location:\tX ");
      Serial.print(integerValue);

      header = Serial1.read();
      if (header == 'Y') {
        String DATA_Y;
        DATA_Y = Serial1.readStringUntil('\n');  // Read the data until a '\n' is encountered
        // Convert the ASCII integer to an int
        integerValue = atoi(DATA_Y.c_str());  // Convert the String to a char array
        if (TargetMode && (abs(integerValue)>5))Tilt_Move(constrain(int(integerValue),-1,2));

        Serial.print("\tY ");
        Serial.println(integerValue);
      }
    } else if (header == 'U') {
      String DATA_U;
      DATA_U = Serial1.readStringUntil('\n');  // Read the data until a '\n' is encountered
      Serial.println("Received Command: Up");
      integerValue = atoi(DATA_U.c_str());  // Convert the String to a char array
      Tilt_Move(-integerValue);

    } else if (header == 'D') {
      String DATA_D;
      DATA_D = Serial1.readStringUntil('\n');  // Read the data until a '\n' is encountered
      Serial.println("Received Command: Down");
      integerValue = atoi(DATA_D.c_str());  // Convert the String to a char array
      Tilt_Move(integerValue);

    } else if (header == 'R') {
      String DATA_R;
      DATA_R = Serial1.readStringUntil('\n');  // Read the data until a '\n' is encountered
      Serial.println("Received Command: Right");
      integerValue = atoi(DATA_R.c_str());  // Convert the String to a char array
      Pan_Move(integerValue);

    } else if (header == 'L') {
      String DATA_L;
      DATA_L = Serial1.readStringUntil('\n');  // Read the data until a '\n' is encountered
      Serial.println("Received Command: Left");
      integerValue = atoi(DATA_L.c_str());  // Convert the String to a char array
      Pan_Move(-integerValue);

    } else if (header == 'S') {
      Serial.println("Received Command: SHOOT");
      Launch();

    } else if (header == 'T') {
      TargetMode = !TargetMode;
      Serial.println("Received Command: Targetmode = " + String(TargetMode));
    }
  }
}

// ================================================================
// ===                       Main Setup                         ===
// ================================================================
void setup() {
  INIT_IO();
  INIT_Serial();
  INIT_Serial1();
  INIT_Motors();
  Home_TiltAxis();
  INIT_LED_rgb();
  INIT_MPU();
  Pan_PID_Setup();
  Tilt_PID_Setup();
  Serial.println("System Ready!");
}
// ================================================================
// ===                        Main Loop                         ===
// ================================================================

void loop() {
  LED_Rainbow();

  Loop_MPU();

  Serial_USB();  // Check for new serial commands
  Serial_Camera();

  Pan_Target = constrain(Pan_Target, -120, 120);
  Tilt_Target = constrain(Tilt_Target, 0, 80);
  PanControl(Pan_Target);
  TiltControl(Tilt_Target);

  LaunchControl();


  Serial.println("Motor Ouput (Tilt : Pan) =\t" + String(Tilt_Output) + "\t:\t" + String(Pan_Output));
}
