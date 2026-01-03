#include <Bluepad32.h>
#include <ODriveUART.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>

//uint8_t
#define BLUE_LED 2
#define RED_LED 4
#define GREEN_LED 5

const uint8_t JOYSTICK_X_AXIS_PIN = 34;
const uint8_t JOYSTICK_Y_AXIS_PIN = 35;

#define JOYSTICK_FORWARD_GPIO 14
#define JOYSTICK_BACKWARD_GPIO 27
#define JOYSTICK_LEFT_GPIO 12
#define JOYSTICK_RIGHT_GPIO 13
#define SWITCH_JOYSTICK_GPIO 32
#define SWITCH_BLUETOOTH_GPIO 33
const uint8_t LIGHT_PWM_GPIO=26; // blue 
#define LIGHT_TOGGLE_GPIO 15
#define UNUSED_GPIO 25 // blue/white

#define ODRIVE_RX_GPIO 16
#define ODRIVE_TX_GPIO 17

// Serial 1 mapped to pins 22/23 for modbus 
const uint8_t LEG_RX_PIN = 22;
const uint8_t LEG_TX_PIN = 23;


// State directly maps to signaling scheme as a bitmask
// red: 1
// green: 2
// blue: 4
// flashing: 8
#define STATE_UNDEFINED 9           // red flashing
#define STATE_IDLE 1                // red
#define STATE_JOYSTICK_DRIVE 2      // green
#define STATE_BLUETOOTH_DRIVE 4     // blue
#define STATE_BLUETOOTH_WAITING 12  // blue flashing
#define STATE_ERROR 15              // white flashing
// spare
// magenta  5
// cyan     6


// ### Global state variables ####

int myState = STATE_UNDEFINED;
bool bluetoothAlive = false;
bool legConnected = true;
unsigned long lastBluetoothUpdate;

// ## Global controller objects
#define NUM_GAMEPADS 1
ControllerPtr myController;
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

  float realX = (abs(x) < 15) ? 0.0f : x;
  float realY = (abs(y) < 15) ? 0.0f : y;

  float desiredVel = (MAX_VEL * realY) / 100.0;
  float desiredYaw = (MAX_YAW * -realX) / 100.0;

  float vel_left = VEL_COEF * desiredVel - YAW_COEF * desiredYaw;
  float vel_right = VEL_COEF * desiredVel + YAW_COEF * desiredYaw;

  // return (vel_left * LEFT_DIR, vel_right * RIGHT_DIR)
  *axisSpeed0 = vel_left * LEFT_DIR;
  *axisSpeed1 = vel_right * RIGHT_DIR;

  *legSpeed = ((abs(realX) > 15) || (abs(realY) > 15)) ? LEG_MAX_SPEED : 0.0f;
}

// Controller connected
void onConnectedController(ControllerPtr ctl) {

  if (myController == nullptr) {
    Serial.printf("CALLBACK: Controller is connected");
    // Additionally, you can get certain gamepad properties like:
    // Model, VID, PID, BTAddr, flags, etc.
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);

    if (ctl->isGamepad()) {
      myController = ctl;
    }
  } else {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("CALLBACK: Controller disconnected");
  myController = nullptr;
}


void readBluetoothJoystick(int32_t* x, int32_t* y) {
  TickType_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  bool bluetoothDataUpdated;

  bluetoothDataUpdated = BP32.update();
  if (bluetoothDataUpdated) {


    // Also show state on Joycon, if possible
    int toggle = millis() / 1000;
    int blink = toggle % 4; 
    myController->setPlayerLEDs(1 << blink); 

    unsigned long delta = millis() - lastBluetoothUpdate;
    Serial.printf("D:%d\n", delta);

    if(delta < 1000) {
      bluetoothAlive = true;
          
      /*uint8_t ledMask = (myState == 1) ? 1 :
                        (myState == 2) ? 3 : 
                        (myState == 4) ? 7 : 0;
                        */
    } else {
      bluetoothAlive = false;
    }


    lastBluetoothUpdate = millis();
  }

  if(bluetoothAlive) {

    int accel = myController->axisX(); // reversed!
    int steer = myController->axisY();
    
    // "calibrated" for white controller
    *y = constrain( map(accel, -407, 292, -100, 100), -100, 100);
    *x = constrain( map(steer, -351, 401, -100, 100), -100, 100);
  } else {
    *x = 0;
    *y = 0;
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
  analogWrite(RED_LED, 255 * redChannel / scale);     // 4000 mCd
  analogWrite(GREEN_LED, 20 * greenChannel / scale);  // 8000 mCd
  analogWrite(BLUE_LED, 150 * blueChannel / scale);   // 5000 mCd

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

  // calibrate
  int32_t xCalib = map(xRaw, 4096, 0, -100, 100);
  int32_t yCalib = map(yRaw, 0, 4096, -100, 134); // for shifting zero


  *x = constrain(xCalib, -100, 100);
  *y = constrain(yCalib, -100, 100);
}



// Arduino setup function. Runs in CPU 1
void setup() {
  // Board LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SWITCH_JOYSTICK_GPIO, INPUT_PULLUP);
  pinMode(SWITCH_BLUETOOTH_GPIO, INPUT_PULLUP);

  // Unused GPIOS
  pinMode(JOYSTICK_FORWARD_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_BACKWARD_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_LEFT_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_RIGHT_GPIO, INPUT_PULLUP);

  pinMode(LIGHT_TOGGLE_GPIO, INPUT_PULLUP);
  pinMode(LIGHT_PWM_GPIO, OUTPUT);



  //xTaskCreate(odriveLoop, "odrive", 2048, NULL, 7, NULL);

  //xTaskCreate(driveTaskFunction, "drive", 4096, NULL, 6, NULL);

  //xTaskCreate(joystickTaskFunction, "joystick", 4096, NULL, 5, NULL);
  /*xTaskCreate(
    displayLoop,  // Function name of the task
    "display",    // Name of the task (e.g. for debugging)
    2048,         // Stack size (bytes)
    NULL,         // Parameter to pass
    1,            // Task priority
    NULL          // Task handle
  );*/
  //xTaskCreatePinnedToCore(displayLoop, "display", 2048, NULL, 1,  NULL, 1);
  


 // xTaskCreate(lightLoop, "light", 2048, NULL, 2, NULL);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);




  //Serial.printf("pdMS %d", pdMS_TO_TICKS(1000));

  //Serial.printf("TICK RATE %d\n", configTICK_RATE_HZ);
  

  pinMode(LEG_RX_PIN, INPUT);
  pinMode(LEG_TX_PIN, OUTPUT);



   // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  //xTaskCreate(bluetoothTaskFunction, "bluetooth", 4096, NULL, 4, NULL);

  //xTaskCreatePinnedToCore(legTaskFunction, "leg", 4096, NULL, 2, NULL, 1); 


  //xTaskCreate(selectTaskFunction, "select", 4096, NULL, 3, NULL); 

  // ODrive
  myOdriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_GPIO, ODRIVE_TX_GPIO);

  Serial1.begin(9600, SERIAL_8N1, LEG_RX_PIN, LEG_TX_PIN);
  legController.begin(1, Serial1);

  uint8_t result = setLegControl(false, LEG_FORWARD);
  legConnected = (result == legController.ku8MBSuccess);

  if(legConnected)
  {
    setLegStartTorque(50);  // 0..255 - 10%
  }
 }  


void selectTaskFunction(void *parameter) {

  while(true) {
    // read inputs
    int switchJoystick = (digitalRead(SWITCH_JOYSTICK_GPIO) == LOW);
    int switchBluetooth = (digitalRead(SWITCH_BLUETOOTH_GPIO) == LOW);

    myState = (switchJoystick) ? STATE_JOYSTICK_DRIVE :
              (switchBluetooth) ? ((bluetoothAlive) ? STATE_BLUETOOTH_DRIVE : STATE_BLUETOOTH_WAITING) :
              STATE_IDLE;

    delay(50);
  }
}


// Arduino loop function. Runs in CPU 1.
void loop() {

  unsigned long cycleStart = millis();

  bool odriveArmed = false;
  bool legArmed = false;

  // Read main rotary switch
  int switchJoystick = (digitalRead(SWITCH_JOYSTICK_GPIO) == LOW);
  int switchBluetooth = (digitalRead(SWITCH_BLUETOOTH_GPIO) == LOW);

  myState = (switchJoystick) ? STATE_JOYSTICK_DRIVE :
            (switchBluetooth) ? ((bluetoothAlive) ? STATE_BLUETOOTH_DRIVE : STATE_BLUETOOTH_WAITING) :
            STATE_IDLE;

  setStatusLED();

  int32_t xJoystick, yJoystick;
  readJoystick(&xJoystick, &yJoystick);

  int32_t xBluetoothJoystick, yBluetoothJoystick;
  readBluetoothJoystick(&xBluetoothJoystick, &yBluetoothJoystick); // need bluetooth read to detect connected controller 

  int32_t xInput = (myState == STATE_JOYSTICK_DRIVE) ? xJoystick :
                   (myState == STATE_BLUETOOTH_DRIVE) ? xBluetoothJoystick : 0;
  int32_t yInput = (myState == STATE_JOYSTICK_DRIVE) ? yJoystick :
                   (myState == STATE_BLUETOOTH_DRIVE) ? yBluetoothJoystick : 0;


  // Effectors

  float axisSpeed0, axisSpeed1, legSpeed;
  drive(xInput, yInput, &axisSpeed0, &axisSpeed1, &legSpeed);

  // Main motor setpoint control
  switch(myState) {
    case STATE_JOYSTICK_DRIVE: 
    case STATE_BLUETOOTH_DRIVE:
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
      case STATE_BLUETOOTH_DRIVE:
      case STATE_JOYSTICK_DRIVE:
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

  unsigned long cycleTime = millis() - cycleStart;

  // Serial plotter
  // Serial.printf("X:%d Y:%d A0:%f A1:%f L:%f T:%d BA:%d\n", xInput, yInput, axisSpeed0, axisSpeed1, legSpeed, cycleTime, bluetoothAlive);
  
  delay(10);
}



/*
// KEPT FOR REFERENCE
void legTaskFunction(void* parameter) {
  //Serial.printf("leg core %d\n", xPortGetCoreID());
  // UART1
  Serial1.begin(9600, SERIAL_8N1, LEG_RX_PIN, LEG_TX_PIN);
 // legSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, LEG_RX_PIN, LEG_TX_PIN);
  //legSerial.enableIntTx(true);
  legController.begin(1, Serial1);

  bool armed = false;
  setLegControl(false, LEG_FORWARD);
  delay(50);
  setLegStartTorque(50);  // 0..255 - 10%
  delay(50);
  // speed
  // setLegTime(20, 0); // 2s to full speed
  // delay(50);

  while(true) {

    switch (myState) {

      //Serial.printf("WTF STATE %d", myState);
      
      case STATE_IDLE: // probably remove that in the future
      case STATE_BLUETOOTH_DRIVE:
      case STATE_JOYSTICK_DRIVE:
        if (!armed) {
          setLegControl(true, LEG_FORWARD);
          armed = true;
        }
        // tbd: make this dependent on drive speed (i.e. joystick)
        if(lightState) {
          setLegSpeed(LEG_MAX_SPEED);
        } else {
          setLegSpeed(0);
        }
        break;

      default:
        if(armed) {
          setLegControl(false, LEG_FORWARD);
          armed=false;
        }
    }
 
    delay(500);
  }
}
*/


/*

// KEPT FOR REFERENCE

// drive task
// read input -> aggregate/compute requested speed per axis
void driveTaskFunction(void *parameter) {
  // important we are called really cyclic
  TickType_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  while (true) {
    switch (myState) {
      case STATE_BLUETOOTH_DRIVE:
      case STATE_JOYSTICK_DRIVE:

        // get input in atomic read
        int32_t input = driveInput;

        // current joystick/controller input
        //  4  3  2
        //  1  0 -1
        // -2 -3 -4
        // we could have used a bitmask, but balanced ternary is just tempting
        int forward = (input > 1);
        int backward = (input < -1);
        input = input + (forward ? -3 : 0) + (backward ? 3 : 0);
        // if we go backward, then swap left/right
        int left = backward ? (input < 0) : (input > 0);
        int right = backward ? (input > 0) : (input < 0);



        float desired_vel = (forward ? MAX_VEL : 0.0f) + (backward ? -MAX_VEL : 0.0f);
        float desired_yaw = (left ? MAX_YAW : 0.0f) + (right ? -MAX_YAW : 0.0f);

        float error_vel = desired_vel - currentVel;
        // todo compute if we apply different accelations (brake or no)
        bool brake = (!forward && !backward)                            // stop
                     || ((currentVel > 0.0f) && (desired_vel < 0.0f))   // or currently forward -> requested backward
                     || ((currentVel < 0.0f) && (desired_vel > 0.0f));  // or currently backward -> requested forward

        float delta_vel = (brake) ? constrain(error_vel, -VEL_ACCEL_BRAKE_CYCLE, VEL_ACCEL_BRAKE_CYCLE) : constrain(error_vel, -VEL_ACCEL_CYCLE, VEL_ACCEL_CYCLE);
        currentVel = currentVel + delta_vel;

        float delta_yaw = constrain(desired_yaw - currentYaw, -YAW_ACCEL_CYCLE, YAW_ACCEL_CYCLE);
        currentYaw = currentYaw + delta_yaw;

        // NOW vel/yaw to axis
        // https://github.com/odriverobotics/ODriveResources/blob/master/botwheel-explorer/bot_ctrl.py#L134
        float vel_left = VEL_COEF * currentVel - YAW_COEF * currentYaw;
        float vel_right = VEL_COEF * currentVel + YAW_COEF * currentYaw;

        // return (vel_left * LEFT_DIR, vel_right * RIGHT_DIR)
        requestedAxisSpeed[0] = vel_left * LEFT_DIR;
        requestedAxisSpeed[1] = vel_right * RIGHT_DIR;

       // Serial.printf("wheelz V[%+6.1f %+6.1f] Y[%+8.1f %+8.1f] B[%d] A[%+5.1f %+5.1f]\n", desired_vel, currentVel, desired_yaw, currentYaw, brake, requestedAxisSpeed[0], requestedAxisSpeed[1]);

        break;
    }
    vTaskDelayUntil(&lastWakeTime, DRIVE_TASK_CYCLE_PERIOD);
  }
}
*/
/*
// KEPT for reference
void odriveLoop(void *parameter) {
  bool armed = false;

  // ODrive
  myOdriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_GPIO, ODRIVE_TX_GPIO);

  while (true) {
    switch (myState) {
      case STATE_ERROR:
        if (armed) {
          odrive.setDualState(AXIS_STATE_IDLE);
          odrive.setDualVelocity(0.0f, 0.0f);
          armed = false;
        }
        break;
      case STATE_IDLE:
        if (armed) {
          odrive.setDualState(AXIS_STATE_IDLE);
          odrive.setDualVelocity(0.0f, 0.0f);
          odrive.clearErrors();
          armed = false;
        }
        break;

      case STATE_BLUETOOTH_WAITING:
        if (armed) {
          odrive.setDualState(AXIS_STATE_IDLE);
          odrive.setDualVelocity(0.0f, 0.0f);
          armed = false;
        }
        break;

      case STATE_BLUETOOTH_DRIVE:
      case STATE_JOYSTICK_DRIVE:
        if (!armed) {
          odrive.setDualState(AXIS_STATE_CLOSED_LOOP_CONTROL);
          armed = true;
        }
        odrive.setDualVelocity(requestedAxisSpeed[0], requestedAxisSpeed[1]);
        break;
    }

    vTaskDelay(50);


//    Serial.print("DC voltage: ");
  //  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

  }
}
*/