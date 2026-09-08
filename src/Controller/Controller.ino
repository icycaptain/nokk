#include <Arduino.h>
#include <ODriveUART.h>
#include <HardwareSerial.h>
#include <PS2X_lib.h>

#include <Dmx_ESP32.h>

namespace ts {
  #include <TaskScheduler.h>
}

#include "AudioTools.h"
#include "FS.h"
#include "LittleFS.h"

// PIN ASSIGNMENT
const uint8_t BLUE_LED_PIN = 2;
const uint8_t RED_LED_PIN = 4;
const uint8_t GREEN_LED_PIN = 15;

const uint8_t JOYSTICK_X_AXIS_PIN = 34;
const uint8_t JOYSTICK_Y_AXIS_PIN = 35;

const uint8_t SWITCH_LOCAL_PIN = 12;
const uint8_t SWITCH_REMOTE_PIN = 13;

const uint8_t TOGGLE_PIN = 21; // generic toggle

const uint8_t ODRIVE_RX_PIN = 16; // implicit by using UART2
const uint8_t ODRIVE_TX_PIN = 17; // implicit by using UART2

// MAX485
const uint8_t DMX_TX_PIN = 14;   // Serial1 MAX485
const uint8_t DMX_RX_PIN = 27; // unused

// I2S Audio
const uint8_t I2S_DIN_PIN = 33;   
const uint8_t I2S_LRC_PIN = 32;   
const uint8_t I2S_BCLK_PIN = 25;  

// PS2 controller
const uint8_t PS2_DAT_PIN = 19;  //MISO  19
const uint8_t PS2_CMD_PIN = 23;  //MOSI  23
const uint8_t PS2_SEL_PIN = 5;  //SS     5
const uint8_t PS2_CLK_PIN = 18;  //SLK   18


// State directly maps to signaling scheme as a bitmask
// red: 1
// green: 2
// blue: 4
// flashing: 8
const uint8_t STATE_UNDEFINED = 9;           // red flashing
const uint8_t STATE_IDLE = 1;                // red
const uint8_t STATE_LOCAL_DRIVE = 2;      // green
const uint8_t STATE_REMOTE_DRIVE = 4;     // blue
const uint8_t STATE_ERROR = 15;              // white flashing
// spare
// magenta  5
// cyan     6


// ### Global state variables ####
int myState = STATE_UNDEFINED;

bool globalLightSwitch = false;  // global switch for light output




// ###############################

// ## Global controller objects
PS2X ps2x;
byte controllerType = 0;

HardwareSerial myOdriveSerial(2); // RX:16, TX: 17
ODriveUART odrive(myOdriveSerial);

//const int REMOTE_VEL_LIMIT = 100; // for load testing
//const int REMOTE_YAW_LIMIT = 100; // for load testing

const int REMOTE_VEL_LIMIT = 50; // % remote controller speed limit compared to local drive (lower than 100)
const int REMOTE_YAW_LIMIT = 75; // % remote controller yawd limit compared to local drive (lower than 100)

// Vehicle properties
const float MAX_VEL = 2.22f;   // m/s (8 km/h)
const float MAX_YAW = 0.8f;    // turns/s
const float WHEEL_DIAMETER = 0.2159f;  // m 
const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0f; // m

const float WHEEL_SPACING = 1.0f; // m
const float CENTER_DISTANCE = WHEEL_SPACING / 2.0f; // m


const float WHEEL_PERIMETER = 678.3f;  // mm   // Pi() * 215.9
const float VEL_COEF = 1000 / WHEEL_PERIMETER;
const float YAW_COEF = WHEEL_SPACING / WHEEL_PERIMETER;
const float LEFT_DIR = -1.0f;
const float RIGHT_DIR = 1.0f;

const float MAX_VEHICLE_THRUST = 90.0f; // N  (divide by vehicle weight to get accelleration   
const float MAX_VEHICLE_TORQUE = 15.0f;  // Nm (divide by vehicle inertia for get rotational acceleation 

/*
// Leg Speed
const float LEG_MAX_SPEED = 0.5f; // half rotation per second
const bool LEG_FORWARD = true;
*/

#define DMX_PORT &Serial1
#define TX_ENABLE -1
dmxTx dmxSend(DMX_PORT, DMX_TX_PIN, TX_ENABLE, -1, LOW); // , LED_GREEN, LOW);


I2SStream i2s;
VolumeStream volume(i2s); 
File audioFile;
WAVDecoder decoder;
StreamCopy copier(volume, audioFile); 

// files on LittleFS 
const char *audioFilenames[] = {"/R2D2-yeah.wav", "/swvader02.wav", "/blaster-firing.wav", "/chewbacca.wav"};

int audioFileSelected = 0;
bool playAudio = false;

// 
const unsigned long LIGHT_TASK_CYCLE_TIME = 40; 
ts::Task lightTask(LIGHT_TASK_CYCLE_TIME, TASK_FOREVER, &lightLoop);

const unsigned long DMX_TASK_CYCLE_TIME = 30; 
ts::Task dmxTask(DMX_TASK_CYCLE_TIME, TASK_FOREVER, &dmxLoop);

const unsigned long DRIVE_TASK_CYCLE_TIME = 100; 
ts::Task driveTask(DRIVE_TASK_CYCLE_TIME, TASK_FOREVER, &driveLoop);

const unsigned long AUDIO_TASK_CYCLE_TIME = 10; 
ts::Task audioTask(AUDIO_TASK_CYCLE_TIME, TASK_FOREVER, &audioLoop);


ts::Scheduler runner;



// Map X/Y Input to setpoints for axis0/axis1/leg
void driveTorque(int32_t x, int32_t y, float* axisTorque0, float* axisTorque1, float *legSpeed) {

  // Zero position threshold: filter mimimum joystick movement
  float realX = (abs(x) < 15) ? 0.0f : x;
  float realY = (abs(y) < 15) ? 0.0f : y;

  // we normalize realX/Y values into a 100 unit circle (while turning, there is no need for full speed)
  float length = sqrt(sq(realX) + sq(realY));
  float scale = (length > 100) ? 100.0/length : 1.0;
  float normalX = scale*realX;
  float normalY = scale*realY;
  
  // compute to overall ehicle thrust/torque
  float vehicleThrust = (MAX_VEHICLE_THRUST * normalY) / 100.0;
  float vehicleTorque = (MAX_VEHICLE_TORQUE * -normalX) / 100.0;

  // translate to individual axis forces
  float thrustLeft = 0.5*vehicleThrust - vehicleTorque / CENTER_DISTANCE;
  float thrustRight = 0.5*vehicleThrust + vehicleTorque / CENTER_DISTANCE;
  
  // translate to individual axis torque
  float torqueLeft = thrustLeft * WHEEL_RADIUS;
  float torqueRight = thrustRight * WHEEL_RADIUS;

  // return (vel_left * LEFT_DIR, vel_right * RIGHT_DIR)
  *axisTorque0 = torqueLeft * LEFT_DIR;
  *axisTorque1 = torqueRight * RIGHT_DIR;

  //Serial.printf("F:%f M:%f F0:%f F1:%f M0:%f M1:%f\n", vehicleThrust, vehicleTorque, thrustLeft, thrustRight, torqueLeft, torqueRight);


  /* 
    *legSpeed = ((abs(realX) > 15) || (abs(realY) > 15)) ? LEG_MAX_SPEED : 0.0f;
   */
}


void readRemoteJoystick(int32_t* x, int32_t* y) {
  if(controllerType == 1){ //DualShock Controller
    ps2x.read_gamepad(); // false, 0); // vibrate); //read controller and set large motor to spin at 'vibrate' speed

    uint8_t xRaw = ps2x.Analog(PSS_RX);
    uint8_t yRaw = ps2x.Analog(PSS_RY);

    if (ps2x.ButtonPressed(PSB_PAD_UP)) {
      audioFileSelected = 0;
      playAudio = true;
    }

    if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
      audioFileSelected = 1;
      playAudio = true;
    }
    
    if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
      audioFileSelected = 2;
      playAudio = true;
    }

    if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
      audioFileSelected = 3;
      playAudio = true;
    }

    if (ps2x.ButtonPressed(PSB_L2) || ps2x.ButtonPressed(PSB_R2)) {
      globalLightSwitch = !globalLightSwitch;
    }

    *x = map(xRaw, 0, 255, -REMOTE_YAW_LIMIT, REMOTE_YAW_LIMIT); // contrain not needed because its a uint8
    *y = map(yRaw, 0, 255, REMOTE_VEL_LIMIT, -REMOTE_VEL_LIMIT); // invert
  }
}

// Set Status LED
void setStatusLED() {
  
  // indicate state using led colors
  int redChannel = (myState >> 0) & 1;
  int greenChannel = (myState >> 1) & 1;
  int blueChannel = (myState >> 2) & 1;
  int flashing = (myState >> 3) & 1;
  int scale = (redChannel + greenChannel + blueChannel);

  if (flashing && ((millis() / 500) & 1)) {
    redChannel = 0.0f;
    greenChannel = 0.0f;
    blueChannel = 0.0f;
  }
  analogWrite(RED_LED_PIN, 255 * redChannel / scale);     // 4000 mCd
  analogWrite(GREEN_LED_PIN, 20 * greenChannel / scale);  // 8000 mCd
  analogWrite(BLUE_LED_PIN, 150 * blueChannel / scale);   // 5000 mCd
}

void readJoystick(int32_t* x, int32_t* y) {
  uint16_t xRaw = analogRead(JOYSTICK_X_AXIS_PIN);
  uint16_t yRaw = analogRead(JOYSTICK_Y_AXIS_PIN);

  //Serial.printf("xr: %d yr: %d\n", xRaw, yRaw);

  // calibrate
  int32_t xCalib = map(xRaw, 4096, 0, -100, 100);
  int32_t yCalib = map(yRaw, 0, 4096, -100, 134); // for shifting zero

  *x = constrain(xCalib, -100, 100);
  *y = constrain(yCalib, -100, 100);
}

/*
void turnOnLeg() {
  uint8_t cmd[] = {0x01, 0x06, 0x80, 0x00, 0x0B, 0x04, 0xA6, 0xF9};
  Serial1.write(cmd, 8);
  delay(5);
}

void turnOffLeg() {
  uint8_t cmd[] = {0x01, 0x06, 0x80, 0x00, 0x0A, 0x04, 0xA7, 0x69};
  Serial1.write(cmd, 8);
  delay(5);
}

void setTorqueLeg() {
  uint8_t cmd[] = {0x01, 0x06, 0x80, 0x02, 0x32, 0x00, 0x14, 0xAA};
  Serial1.write(cmd, 8);
  delay(5);
}

void setSpeedLeg() {
  uint8_t cmd[] = {0x01, 0x06, 0x80, 0x05, 0xB0, 0x04, 0xC4, 0x08};
  Serial1.write(cmd, 8);
  delay(5);
}
*/

void setup() {
  Serial.begin(115200);

  // Board LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(SWITCH_LOCAL_PIN, INPUT_PULLUP);
  pinMode(SWITCH_REMOTE_PIN, INPUT_PULLUP);

  pinMode(TOGGLE_PIN, INPUT_PULLUP);

  int tryNum = 0;
  int error = -1;
  while (error != 0) {
    delay(1000);// 1 second wait
    //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, false, false);
    Serial.printf("error %d", error);
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum ++;
  }
  controllerType = ps2x.readType();
  switch(controllerType) {
    case 0:
      Serial.println(" Unknown Controller type found ");
      break;
    case 1:
      Serial.println(" DualShock Controller found ");
      break;
    case 2:
      Serial.println(" GuitarHero Controller found ");
      break;
	  case 3:
      Serial.println(" Wireless Sony DualShock Controller found ");
      break;
  }

  // ODrive
  myOdriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
 
 /*
  //pinMode(LEG_RX_PIN, INPUT);
  //pinMode(LEG_TX_PIN, OUTPUT);
  Serial1.begin(9600, SERIAL_8N1, LEG_RX_PIN, LEG_TX_PIN);
  turnOffLeg();
  setTorqueLeg();
  setSpeedLeg();
  */

  Serial1.begin(250000, SERIAL_8N2, -1, DMX_TX_PIN);
  if (!dmxSend.configure()) {
    Serial.println("Error: Cannot configure DMX on Serial1");
  } else {
    Serial.println("DMX configured on Serial1.");
  }


  // audio init
  if(!LittleFS.begin()){
    Serial.println("LittleFS Mount Failed!");
    return;
  }

  // I2S für Stereo initialisieren
  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.port_no = 1; 
  cfg.pin_bck = I2S_BCLK_PIN;  
  cfg.pin_ws = I2S_LRC_PIN;    
  cfg.pin_data = I2S_DIN_PIN;   
  cfg.sample_rate = 11025; // Bei Bedarf auf 44100 ändern
  cfg.channels = 2;        
  cfg.bits_per_sample = 16;
  i2s.begin(cfg);

  // Lautstärke einstellen
  volume.begin(cfg); 
  volume.setVolume(1.0); //0.2); 

  runner.addTask(driveTask);
  driveTask.enable();
  runner.addTask(lightTask);
  lightTask.enable();
  runner.addTask(dmxTask);
  dmxTask.enable();
  runner.addTask(audioTask);
  audioTask.enable();

}  

bool odriveArmed = false;

void loop() {
  runner.execute();
  delay(1);
}


void driveLoop() {

  // Read main rotary switch
  int switchLocal = (digitalRead(SWITCH_LOCAL_PIN) == LOW);
  int switchRemote = (digitalRead(SWITCH_REMOTE_PIN) == LOW);

  myState = (switchLocal) ? STATE_LOCAL_DRIVE :
            (switchRemote) ? STATE_REMOTE_DRIVE :
            STATE_IDLE;

  setStatusLED();

  static int lastToggleRead = HIGH;
  int currentToggleRead = digitalRead(TOGGLE_PIN);
  if((currentToggleRead == LOW) && (lastToggleRead == HIGH)) {
    // "rising edge"
    globalLightSwitch = !globalLightSwitch;
  }
  lastToggleRead = currentToggleRead;
  
  int32_t xLocalJoystick, yLocalJoystick;
  readJoystick(&xLocalJoystick, &yLocalJoystick);

  int32_t xRemoteJoystick, yRemoteJoystick;
  readRemoteJoystick(&xRemoteJoystick, &yRemoteJoystick); // need read to keep controller updated 

  int32_t xInput = (myState == STATE_LOCAL_DRIVE) ? xLocalJoystick :
                   (myState == STATE_REMOTE_DRIVE) ? xRemoteJoystick : 0;
  int32_t yInput = (myState == STATE_LOCAL_DRIVE) ? yLocalJoystick :
                   (myState == STATE_REMOTE_DRIVE) ? yRemoteJoystick : 0;


  // Effectors
  //float axisSpeed0, axisSpeed1, legSpeed;
  float axisTorque0, axisTorque1, legSpeed;
  
  //drive(xInput, yInput, &axisSpeed0, &axisSpeed1, &legSpeed);
  driveTorque(xInput, yInput, &axisTorque0, &axisTorque1, &legSpeed);

  // Main motor setpoint control
  switch(myState) {
    case STATE_LOCAL_DRIVE: 
    case STATE_REMOTE_DRIVE:
      if(!odriveArmed) {
        odrive.setDualState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveArmed = true;
      }
      // odrive.setDualVelocity(axisSpeed0, axisSpeed1);
      
      // SVEN DEBUG
      odrive.setDualTorque(axisTorque0, axisTorque1);
      break;
    default:
      if (odriveArmed) {
        odrive.setDualState(AXIS_STATE_IDLE);
        //odrive.setDualVelocity(0.0f, 0.0f);
        odrive.setDualTorque(0.0f, 0.0f);

        odrive.clearErrors();
        odriveArmed = false;
      }
  }

  /* Leg setpoint control
  switch (myState) {
    case STATE_LOCAL_DRIVE:
    case STATE_REMOTE_DRIVE:
      if(legSpeed > 0) {
        turnOnLeg();
      }
      else 
      {
        turnOffLeg();
      }
      break;
    default:
      turnOffLeg();
  }
  */
  /*
  dmxSend.write(0, 1);
  dmxSend.write(0, 2);
  dmxSend.write(0, 3);
  dmxSend.write(0, 4);
  */

  
 // vTaskDelayUntil(&lastWakeTime, 100); // 100ms cycle
}


#define NUM_COLORS 4
#define NUM_CHANNELS 4
const uint8_t COLORS[NUM_COLORS][NUM_CHANNELS] = {
 // { 148, 0,  211, 0}, // violet
  { 30, 10, 255, 0}, // violet
  { 255, 20, 147, 0}, // pink
  { 0,   0,  255, 0}, // blue
  { 255, 69, 0,   0}  // orange
};


uint8_t lightOutput[NUM_CHANNELS];

const int lightPeriod = 100; // in cycles , not ms
const int fadePeriod = lightPeriod / NUM_COLORS;

// LIGHT_TASK_CYCLE_TIMEis 40ms
void lightLoop() {

  static float masterFade = 0.0;
  static int t = 0; // tick counter
  
  t++;
  if(t >= lightPeriod)  {
    t = 0;
  }

  //masterFade = constrain( (globalLightSwitch) ? masterFade + 0.08 : masterFade - 0.08, 0.0, 1.0);
  masterFade = (globalLightSwitch) ? 1.0 : 0.0;

  int fadeStep = t / fadePeriod;
  int microStep = t - fadeStep * fadePeriod;
  float fade = 0.5 * (cos( PI * microStep / fadePeriod ) + 1); 
  for(int channel = 0; channel < NUM_CHANNELS; channel++) {
    lightOutput[channel] = (uint8_t) (masterFade * (fade * COLORS[fadeStep][channel] + (1-fade) * COLORS[(fadeStep+1)%NUM_COLORS][channel]));
  }
}




// Task
// read light and other channels and send to DMX
void dmxLoop() {
  for(int channel = 0; channel < NUM_CHANNELS; channel++) {
    dmxSend.write(lightOutput[channel], 1+channel);
  }
  dmxSend.transmit();
}


void audioLoop() {
  if(playAudio) {

    if(!audioFile) {
      // Loading audiofile
      audioFile = LittleFS.open(audioFilenames[audioFileSelected], "r");
      if (!audioFile) {
        return;
      }
      copier.begin(volume, audioFile);
    } 

    // Streaming audio file while there is data left
    if (audioFile && audioFile.available()) {
      copier.copy();
    } else {
      if (audioFile) audioFile.close();
      playAudio = false;
    }
  }
}
