#include <ODriveUART.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <PS2X_lib.h>

// PIN ASSIGNMENT
const uint8_t BLUE_LED_PIN = 2;
const uint8_t RED_LED_PIN = 4;
const uint8_t GREEN_LED_PIN = 15;

const uint8_t JOYSTICK_X_AXIS_PIN = 34;
const uint8_t JOYSTICK_Y_AXIS_PIN = 35;

const uint8_t SWITCH_LOCAL_PIN = 12;
const uint8_t SWITCH_REMOTE_PIN = 13;

const uint8_t UNUSED_OUTPUT_PIN = 25; // blue/white
const uint8_t LIGHT_PWM_PIN = 26; // blue 
const uint8_t TOGGLE_PIN = 14; // generic toggle


const uint8_t ODRIVE_RX_PIN = 16; // implicit by using UART2
const uint8_t ODRIVE_TX_PIN = 17; // implicit by using UART2

// Serial 1 mapped to pins for modbus
const uint8_t LEG_RX_PIN = 27;
const uint8_t LEG_TX_PIN = 14;

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
bool legConnected = true;

// ## Global controller objects
//#define NUM_GAMEPADS 1
//ControllerPtr myController = nullptr;
PS2X ps2x;
byte controllerType = 0;

HardwareSerial myOdriveSerial(2); // RX:16, TX: 17
ODriveUART odrive(myOdriveSerial);
ModbusMaster legController;

// Vehicle properties
const float MAX_VEL = 2.22f;   // m/s (8 km/h)
const float MAX_YAW = 0.8f;    // turns/s
const float WHEEL_PERIMETER = 678.3f;  // mm   // Pi() * 215.9
const float WHEEL_SPACING = 1000.0f;   // mm
const float VEL_COEF = 1000 / WHEEL_PERIMETER;
const float YAW_COEF = WHEEL_SPACING / WHEEL_PERIMETER;
const float LEFT_DIR = -1.0f;
const float RIGHT_DIR = 1.0f;

// Leg Speed
const float LEG_MAX_SPEED = 0.5f; // half rotation per second
const bool LEG_FORWARD = true;

// Map X/Y Input to setpoints for axis0/axis1/leg
void drive(int32_t x, int32_t y, float* axisSpeed0, float* axisSpeed1, float *legSpeed) {

  // Zero position threshold: filter mimimum joystick movement
  float realX = (abs(x) < 15) ? 0.0f : x;
  float realY = (abs(y) < 15) ? 0.0f : y;

  // we normalize realX/Y values into a 100 unit circle (while turning, there is no need for full speed)
  float length = sqrt(sq(realX) + sq(realY));
  float scale = (length > 100) ? 100.0/length : 1.0;
  float normalX = scale*realX;
  float normalY = scale*realY;
  
  float desiredVel = (MAX_VEL * normalY) / 100.0;
  float desiredYaw = (MAX_YAW * -normalX) / 100.0;

  float vel_left = VEL_COEF * desiredVel - YAW_COEF * desiredYaw;
  float vel_right = VEL_COEF * desiredVel + YAW_COEF * desiredYaw;

  // return (vel_left * LEFT_DIR, vel_right * RIGHT_DIR)
  *axisSpeed0 = vel_left * LEFT_DIR;
  *axisSpeed1 = vel_right * RIGHT_DIR;

  *legSpeed = ((abs(realX) > 15) || (abs(realY) > 15)) ? LEG_MAX_SPEED : 0.0f;
}

void readRemoteJoystick(int32_t* x, int32_t* y) {
  if(controllerType == 1){ //DualShock Controller
    ps2x.read_gamepad(); // false, 0); // vibrate); //read controller and set large motor to spin at 'vibrate' speed

    uint8_t xRaw = ps2x.Analog(PSS_RX);
    uint8_t yRaw = ps2x.Analog(PSS_RY);

    *x = map(xRaw, 0, 255, -100, 100); // contrain not needed because its a uint8
    *y = map(yRaw, 0, 255, 100, -100); // invert
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

// we do the forward/backward logic here
// nokk forward -> motor backward
// nokk backward -> motor forward
uint8_t setLegControl(bool enable, bool forward) {
  const uint16_t pole_pairs = 0x0004; // however  specs says 4 pole pairs
  const bool brake = false;
  const uint16_t ctrl = 
    ((enable)  ? 0x0100 : 0x0000) |
    ((forward) ? 0x0200 : 0x0000) | 
    ((brake)   ? 0x0400 : 0x0000) |
    0x0800; // always RS485

  const uint16_t value = ctrl | pole_pairs;

  //Serial.printf("LEG CTRL %x ", value);
  return legController.writeSingleRegister(0x8000, value);
  //Serial.printf("RES %x\n", result);
}

uint8_t setLegStartTorque(uint8_t torque) {
  return legController.writeSingleRegister(0x8002, (torque << 8) | (0x00));
}

uint8_t setLegTime(uint8_t acceleration, uint8_t deceleration) {
  return legController.writeSingleRegister(0x8003, (acceleration << 8) | (deceleration));
}

uint8_t setLegSpeed(float rot_per_s) {
  // chainwheel has 10/40 ratio
  // planetary gearbox as 1:10 ratio
  const float gearing = (10.0f / 40.0f) * (1.0f / 10.0f);

  // theoretisch müssten sein:
  // 0,5 rps - 1200 RPM
  // 1 rps - 2400 RPM
  uint16_t rpm_motor = (uint16_t) (60.0f * rot_per_s / gearing);

  const uint16_t value =  __builtin_bswap16(rpm_motor);
  uint8_t result = legController.writeSingleRegister(0x8005, value);
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

void setup() {
  Serial.begin(115200);

  // Board LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(SWITCH_LOCAL_PIN, INPUT_PULLUP);
  pinMode(SWITCH_REMOTE_PIN, INPUT_PULLUP);

  pinMode(TOGGLE_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PWM_PIN, OUTPUT);

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

  pinMode(LEG_RX_PIN, INPUT);
  pinMode(LEG_TX_PIN, OUTPUT);
  Serial1.begin(9600, SERIAL_8N1, LEG_RX_PIN, LEG_TX_PIN);
  legController.begin(1, Serial1);

  uint8_t result = setLegControl(false, LEG_FORWARD);
  legConnected = (result == legController.ku8MBSuccess);

  if(legConnected)
  {
    setLegStartTorque(50);  // 0..255 - 10%
  }
}  


void loop() {

  TickType_t lastWakeTime = xTaskGetTickCount();


  bool odriveArmed = false;
  bool legArmed = false;

  // Read main rotary switch
  int switchLocal = (digitalRead(SWITCH_LOCAL_PIN) == LOW);
  int switchRemote = (digitalRead(SWITCH_REMOTE_PIN) == LOW);

  myState = (switchLocal) ? STATE_LOCAL_DRIVE :
            (switchRemote) ? STATE_REMOTE_DRIVE :
            STATE_IDLE;

  setStatusLED();

  int32_t xLocalJoystick, yLocalJoystick;
  readJoystick(&xLocalJoystick, &yLocalJoystick);

  int32_t xRemoteJoystick, yRemoteJoystick;
  readRemoteJoystick(&xRemoteJoystick, &yRemoteJoystick); // need read to keep controller updated 


  int32_t xInput = (myState == STATE_LOCAL_DRIVE) ? xLocalJoystick :
                   (myState == STATE_REMOTE_DRIVE) ? xRemoteJoystick : 0;
  int32_t yInput = (myState == STATE_LOCAL_DRIVE) ? yLocalJoystick :
                   (myState == STATE_REMOTE_DRIVE) ? yRemoteJoystick : 0;


  // Effectors
  float axisSpeed0, axisSpeed1, legSpeed;
  drive(xInput, yInput, &axisSpeed0, &axisSpeed1, &legSpeed);

  // Main motor setpoint control
  switch(myState) {
    case STATE_LOCAL_DRIVE: 
    case STATE_REMOTE_DRIVE:
      if(!odriveArmed) {
        odrive.setDualState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveArmed = true;
      }
      odrive.setDualVelocity(axisSpeed0, axisSpeed1);
      break;
    default:
      if (odriveArmed) {
        odrive.setDualState(AXIS_STATE_IDLE);
        odrive.setDualVelocity(0.0f, 0.0f);
        odrive.clearErrors();
        odriveArmed = false;
      }
  }

  if(legConnected) {
    // Leg setpoint control
    switch (myState) {
      case STATE_LOCAL_DRIVE:
      case STATE_REMOTE_DRIVE:
        if (!legArmed) {
          setLegControl(true, LEG_FORWARD);
          legArmed = true;
        }
        setLegSpeed(legSpeed);
        break;
      default:
        if(legArmed) {
          setLegControl(false, LEG_FORWARD);
          legArmed=false;
        }
        setLegSpeed(0.0f);
    }
  }

  // Serial plotter
  Serial.printf("X:%d Y:%d A0:%f A1:%f L:%f %d\n", xInput, yInput, axisSpeed0, axisSpeed1, legSpeed, legConnected);
  
  vTaskDelayUntil(&lastWakeTime, 100); // 100ms cycle
}
