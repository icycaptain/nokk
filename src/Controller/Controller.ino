#include <Bluepad32.h>
#include <ODriveUART.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>

//uint8_t
#define BLUE_LED 2
#define RED_LED 4
#define GREEN_LED 5

#define JOYSTICK_FORWARD_GPIO 14
#define JOYSTICK_BACKWARD_GPIO 27
#define JOYSTICK_LEFT_GPIO 12
#define JOYSTICK_RIGHT_GPIO 13
#define SWITCH_JOYSTICK_GPIO 32
#define SWITCH_BLUETOOTH_GPIO 33
const uint8_t LIGHT_PWM_GPIO=26; // blue 
#define LIGHT_TOGGLE_GPIO 15

#define UNUSED_GPIO 25 // blue/white

// State directly maps to signaling scheme as a bitmask
// red: 1
// green: 2
// blue: 4
// flashing: 8
#define STATE_UNDEFINED 9          // white flashing
#define STATE_IDLE 1 // 7                // white
#define STATE_JOYSTICK_DRIVE 2      // green
#define STATE_BLUETOOTH_DRIVE 4     // blue
#define STATE_BLUETOOTH_WAITING 12  // blue flashing
#define STATE_ERROR 15 //9               // red flashing
// spare
// magenta  5
// cyan     6

const char *ssid = "VFR_NOKK";
const char *password = "manu1234";

// Task cycles

const TickType_t TELEMETRY_CYCLE_PERIOD = 1000;

// #### Process variables ####

int myState = STATE_UNDEFINED;

bool bluetoothAlive = false;

// current drive input from joystick/controller
//  4  3  2
//  1  0 -1
// -2 -3 -4
int32_t driveInput = 0;

void drive(int forward, int backward, int left, int right) {
  int32_t newInput =
    (forward ? 3 : 0)
    + (backward ? -3 : 0)
    + (left ? 1 : 0)
    + (right ? -1 : 0);

  // atomic state update
  driveInput = newInput;
}


// rotation speed per axis
float requestedAxisSpeed[2] = { 0.0f, 0.0f };
// current ( planned) velocity and yaw
float currentVel = 0.0f;
float currentYaw = 0.0f;

bool lightState = false;

#define NUM_GAMEPADS 1
ControllerPtr myController;
unsigned long lastBluetoothUpdate;

#define ODRIVE_RX_GPIO 16
#define ODRIVE_TX_GPIO 17

HardwareSerial myOdriveSerial(2);
ODriveUART odrive(myOdriveSerial);

const byte LEG_RX_PIN = 22;
const byte LEG_TX_PIN = 23;

ModbusMaster legController;


//
// DRIVE TASK
//
const TickType_t DRIVE_TASK_CYCLE_PERIOD = 40;  // ms = 25Hz

const float MAX_VEL = 2.22f;   // m/s (8 km/h)
const float VEL_ACCEL = 2.0f;  // m/s²
const float VEL_ACCEL_BRAKE = 4.0f;
// acceleration per cycle (period), so in m/s per [period]
const float CYCLE_COEFF = DRIVE_TASK_CYCLE_PERIOD / 1000.0f;
const float VEL_ACCEL_CYCLE = VEL_ACCEL * CYCLE_COEFF;
const float VEL_ACCEL_BRAKE_CYCLE = VEL_ACCEL_BRAKE * CYCLE_COEFF;

const float MAX_YAW = 0.8f;    // turns/s
const float YAW_ACCEL = 0.8f;  // turns/s²
const float YAW_ACCEL_CYCLE = YAW_ACCEL * CYCLE_COEFF;

const float WHEEL_PERIMETER = 678.3f;  // mm   // Pi() * 215.9
const float WHEEL_SPACING = 1000.0f;   // mm
const float VEL_COEF = 1000 / WHEEL_PERIMETER;
const float YAW_COEF = WHEEL_SPACING / WHEEL_PERIMETER;
const float LEFT_DIR = -1.0;
const float RIGHT_DIR = 1.0f;


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

//
// BLUETOOTH TASK
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

/*
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}
*/

/*
  reference

void processGamepad(ControllerPtr ctl) {

  // Sven

  int32_t accel = ctl->axisX();
  int32_t steer = ctl->axisY();

  // drive((accel > threshold), (accel < -threshold), (steer < -threshold), (steer > threshold));


  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
  if (ctl->a()) {
    static int colorIdx = 0;
    // Some gamepads like DS4 and DualSense support changing the color LED.
    // It is possible to change it by calling:
    switch (colorIdx % 3) {
      case 0:
        // Red
        ctl->setColorLED(255, 0, 0);
        break;
      case 1:
        // Green
        ctl->setColorLED(0, 255, 0);
        break;
      case 2:
        // Blue
        ctl->setColorLED(0, 0, 255);
        break;
    }
    colorIdx++;
  }

  if (ctl->b()) {
    // Turn on the 4 LED. Each bit represents one LED.
    static int led = 0;
    led++;
    // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
    // support changing the "Player LEDs": those 4 LEDs that usually indicate
    // the "gamepad seat".
    // It is possible to change them by calling:
    ctl->setPlayerLEDs(led & 0x0f);
  }

  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  //dumpGamepad(ctl);
}
*/

void bluetoothTaskFunction(void *parameter) {
  TickType_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  bool bluetoothDataUpdated;
  int32_t accel, steer;
  const int32_t threshold = 200;

  bool toggleState = false;

  while (true) {
    bluetoothDataUpdated = BP32.update();
    if (bluetoothDataUpdated) {
      lastBluetoothUpdate = millis();

      int secs = millis() / 1000;
      myController->setPlayerLEDs(0x01 << (secs % 4)); 

      bool t = myController->b();
      if(t && !toggleState) {
        // "rising"
        lightState = !lightState;
      }
      toggleState = t;
    }

    if((millis() - lastBluetoothUpdate) < 1000) {
      bluetoothAlive = true;
    } else {
      bluetoothAlive = false;
    }

    switch (myState) {
      case STATE_BLUETOOTH_DRIVE:
        accel = myController->axisX();
        steer = myController->axisY();
        drive((accel > threshold), (accel < -threshold), (steer < -threshold), (steer > threshold));
        break;

      case STATE_BLUETOOTH_WAITING:
        drive(0, 0, 0, 0);
        break;

    }
    //    vTaskDelayUntil(&lastWakeTime, JOYSTICK_CYCLE_PERIOD);
    vTaskDelay(10);
  }
}

void displayLoop(void *parameter) {
  while (true) {
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
    vTaskDelay(100);
  }
}



void lightLoop(void *parameter) {
  // wave flashing 
  const int dc = 150; // avg brightness
  const int amp = 255-dc;
  const TickType_t delay = 50;
  const float freq = 0.5; // Hz
  const float T = 1000/freq;
  const float n = T/delay;
  const float theta_inc = 2*3.14f / n; // increment per cycle

  bool toggleState = false;

  float theta = 0.0f;
  while(true) {

    bool t = digitalRead(LIGHT_TOGGLE_GPIO) == LOW;
    if(t && !toggleState) {
      // "rising"
      lightState = !lightState;
      theta = 0.0f;
    }
    toggleState = t;
    
    // wave
    int v = (int)(cos(theta)*amp + dc);
    theta += theta_inc;

    analogWrite(LIGHT_PWM_GPIO, lightState ? v: 0);

    vTaskDelay(delay);
  }
}

// we do the forward/backward logic here
// nokk forward -> motor backward
// nokk backward -> motor forward
void setLegControl(bool enable, bool forward) {
  const uint16_t pole_pairs = 0x0004; // however  specs says 4 pole pairs
  const bool brake = false;
  const uint16_t ctrl = 
    ((enable)  ? 0x0100 : 0x0000) |
    ((forward) ? 0x0200 : 0x0000) | 
    ((brake)   ? 0x0400 : 0x0000) |
    0x0800; // always RS485

  const uint16_t value = ctrl | pole_pairs;

  //Serial.printf("LEG CTRL %x ", value);
  uint8_t result = legController.writeSingleRegister(0x8000, value);
  //Serial.printf("RES %x\n", result);

}

void setLegStartTorque(uint8_t torque) {
  legController.writeSingleRegister(0x8002, (torque << 8) | (0x00));
}

void setLegTime(uint8_t acceleration, uint8_t deceleration) {
  legController.writeSingleRegister(0x8003, (acceleration << 8) | (deceleration));
}

void setLegSpeed(float rot_per_s) {
  
  // chainwheel has 10/40 ratio
  // planetary gearbox as 1:10 ratio
  const float gearing = (10.0f / 40.0f) * (1.0f / 10.0f);
  //const float rpm_motor = 60.f * rot_per_s / gearing;

  //uint16_t rpm_motor = 600; // ergibt 2 umrehungen pro sekunde ?? 

  // Serial.printf("GEARING: %f\n", gearing);

  // theoretisch müssten sein:
  // 0,5 rps - 1200 RPM
  // 1 rps - 2400 RPM

  uint16_t rpm_motor = (uint16_t) (60.0f * rot_per_s / gearing);


  const uint16_t value =  __builtin_bswap16(rpm_motor);
//  Serial.printf("LEG RPM %d MB %x ", rpm_motor,value);

  uint8_t result = legController.writeSingleRegister(0x8005, value);
//  Serial.printf("RES %x \n", result);
}

const float LEG_MAX_SPEED = 0.5f; // quarter rotation per second
const bool LEG_FORWARD = true;

void legTaskFunction(void* parameter) {
  Serial.printf("leg core %d\n", xPortGetCoreID());
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

      Serial.printf("WTF STATE %d", myState);
      
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

void joystickTaskFunction(void *parameter) {
  TickType_t lastWakeTime;
  //const TickType_t cyclePeriod = 40; // ms = 25Hz
  lastWakeTime = xTaskGetTickCount();

  while (true) {
    if (myState == STATE_JOYSTICK_DRIVE) {
      int forward = digitalRead(JOYSTICK_FORWARD_GPIO) == LOW;
      int backward = digitalRead(JOYSTICK_BACKWARD_GPIO) == LOW;
      int left = digitalRead(JOYSTICK_LEFT_GPIO) == LOW;
      int right = digitalRead(JOYSTICK_RIGHT_GPIO) == LOW;
      drive(forward, backward, left, right);
    }
    vTaskDelay(20);
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {



  // Board LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SWITCH_JOYSTICK_GPIO, INPUT_PULLUP);
  pinMode(SWITCH_BLUETOOTH_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_FORWARD_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_BACKWARD_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_LEFT_GPIO, INPUT_PULLUP);
  pinMode(JOYSTICK_RIGHT_GPIO, INPUT_PULLUP);

  pinMode(LIGHT_TOGGLE_GPIO, INPUT_PULLUP);
  pinMode(LIGHT_PWM_GPIO, OUTPUT);



  xTaskCreate(odriveLoop, "odrive", 2048, NULL, 7, NULL);

  xTaskCreate(driveTaskFunction, "drive", 4096, NULL, 6, NULL);
  xTaskCreate(joystickTaskFunction, "joystick", 4096, NULL, 5, NULL);
  /*xTaskCreate(
    displayLoop,  // Function name of the task
    "display",    // Name of the task (e.g. for debugging)
    2048,         // Stack size (bytes)
    NULL,         // Parameter to pass
    1,            // Task priority
    NULL          // Task handle
  );*/
  xTaskCreatePinnedToCore(displayLoop, "display", 2048, NULL, 1,  NULL, 1);
  


  xTaskCreate(lightLoop, "light", 2048, NULL, 2, NULL);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);




  Serial.printf("pdMS %d", pdMS_TO_TICKS(1000));

  Serial.printf("TICK RATE %d\n", configTICK_RATE_HZ);
  

  pinMode(LEG_RX_PIN, INPUT);
  pinMode(LEG_TX_PIN, OUTPUT);



   // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  //BP32.forgetBluetoothKeys();

  xTaskCreate(bluetoothTaskFunction, "bluetooth", 4096, NULL, 4, NULL);

  xTaskCreatePinnedToCore(legTaskFunction, "leg", 4096, NULL, 3, NULL, 1); 
  //xTaskCreate(legTaskFunction, "leg", 4096, NULL, 666 /*3*/, NULL); 




}



void setState(int newState) {
  myState = newState;
}

// Arduino loop function. Runs in CPU 1.
void loop() {

  // read inputs
  int switchJoystick = (digitalRead(SWITCH_JOYSTICK_GPIO) == LOW);
  int switchBluetooth = (digitalRead(SWITCH_BLUETOOTH_GPIO) == LOW);

  // TODO: switch Joystick and bluetooth must never both on at the same time

  // State changes
  switch (myState) {

    case STATE_UNDEFINED:
    case STATE_ERROR:
      // must have main switch to neutral position first
      if (!switchJoystick && !switchBluetooth) {
        setState(STATE_IDLE);
      }
      break;

    case STATE_IDLE:
      if (switchJoystick) {
        setState(STATE_JOYSTICK_DRIVE);
      } else if (switchBluetooth) {
        if (bluetoothAlive) {
          setState(STATE_BLUETOOTH_DRIVE);
        } else {
          setState(STATE_BLUETOOTH_WAITING);
        }
      }
      break;

    case STATE_JOYSTICK_DRIVE:
      if (!switchJoystick) {
        setState(STATE_IDLE);
      }
      break;

    case STATE_BLUETOOTH_DRIVE:
      if (!switchBluetooth) {
        setState(STATE_IDLE);
      } else if (!bluetoothAlive) {
        setState(STATE_BLUETOOTH_WAITING);
      }
      break;

    case STATE_BLUETOOTH_WAITING:
      if (!switchBluetooth) {
        setState(STATE_IDLE);
      } else if (bluetoothAlive) {
        setState(STATE_BLUETOOTH_DRIVE);
      }


    default:
      break;
  }

  vTaskDelay(10);
}





