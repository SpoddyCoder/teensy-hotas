// Replacement Thrustmaster T.Flight Hotas X controller, using Teensy 3.1, by Pat Daderko (DogP)
// based on Paul Stoffregen's Extreme Joystick Test
// modiupdated + expanded by SpoddyCoder, 2023
// https://github.com/SpoddyCoder/teensy-hotas
// v2.1


/**************************************************
 * Config
 *************************************************/

// serial monitor debug
#if defined(SEREMU_INTERFACE) || (defined(CDC_STATUS_INTERFACE) && defined(CDC_DATA_INTERFACE))
  #define SERIAL_ENABLED
#endif
#define SERIAL_DEBUG 0  // set to true to send useful debug data after initial setup + calibration
#define SERIAL_DEBUG_LOOP 0 // set to true to send raw + filtered analogue values to serial monitor during the main loop

// hardware calibration
#define LOOP_DELAY 5 // ms, throttles the joystick send, eg: 5ms is very approximately 200Hz refresh
#define MIN_AVAL -1400  // range exepected by the controller class (usb_hotasx.h)
#define MAX_AVAL 1400
#define THROTTLE_LIMIT_DEADZONE 0.05  // within % considered min/max
#define THROTTLE_MID_RANGE 0.2  // % of range that lights up the mid-throttle indicator (eg: 10% is >=45% <=55%). Note: this is not a deadzone yet, TODO
#define X_CENTRE_DEADZONE 0.05 // % of range considered 0
#define Y_CENTRE_DEADZONE 0.05
#define RX_CENTRE_DEADZONE 0.0
#define RY_CENTRE_DEADZONE 0.0
#define RZ_CENTRE_DEADZONE 0.15  // because it's easy to apply this accidentally while using the stick
#define S0_CENTRE_DEADZONE 0.05
#define S1_CENTRE_DEADZONE 0.0 

// analog axis filtering
#define FILTER_TYPE 3 // alter to preference...
// 0 = no filter
// 1 = simple moving average
#define SMA_BUF_LEN 16  // number of samples to smooth over
// 2 = one euro filter
#define OEF_MIN_CUTOFF 10 // Hz
#define OEF_D_CUTOFF 8 // Hz
#define OEF_BETA 0.005 // Slope
// 3 = Kalman filter
#define KF_PROCESS_NOISE 0.05  // aka Q, process noise, lower value = more smoothing but more lag
#define KF_MEASUREMENT_NOISE 1  // aka R, measurement noise, higher value = more smoothing but more lag
#define KF_ESTIMATION_ERROR 1  // aka P, estimate error

// button pins
#define NUM_BUTTONS 16 // button pins are 0-12 (+ BACK, PRESET, MAPPING)
#define BACK_SWITCH 28  // PC/PS3
#define PRESET_BTN 27
#define MAPPING_BTN 26
// hat pins
#define HU 19
#define HD 20
#define HL 21
#define HR 22
// analog pins
#define NUM_AXES 8
#define X_AXIS 15  // A1
#define Y_AXIS 16  // A2
#define Z_AXIS 17  // A3
#define RX_AXIS 35 // A11
#define RY_AXIS 36 // A12
#define RZ_AXIS 18 // A4
#define S0_AXIS 34 // A10
#define S1_AXIS 37 // A13
#define DUMMY 23  // A9 - leave disconnected, see axisRead() function for more info
// ...and inversions
#define INV_X 1
#define INV_Y 0
#define INV_Z 1
#define INV_RX 0
#define INV_RY 0
#define INV_RZ 1
#define INV_S0 0
#define INV_S1 0
// enable if you want to wire up the RX, RY and S1 (eg for rudders/toe brakes)
#define ENABLE_ALL_AXIS 0
// LED pins
#define LED_GREEN 14
#define LED_RED 13
#define LED_ORANGE 0 //orange is both green and red LEDs, give it a define for the led function
#define LED_PRESET 24


/**************************************************
 * Filters
 *************************************************/

class SimpleMovingAverage {
  public:
    SimpleMovingAverage(int buffer_len) {
      this->idx = 0;
      this->buffer_len = buffer_len;
      this->buffer = new unsigned short[buffer_len];
      for (int i = 0; i<buffer_len; i++) {
        this->buffer[i] = 0;
      }
    }
  
    ~SimpleMovingAverage() {
      delete[] buffer;
    }

    unsigned short filter(unsigned short value) {
      buffer[this->idx] = value;
      idx = (idx + 1) % this->buffer_len;
      return calculateAverage();
    }

    unsigned short currentAverage() {
      return calculateAverage();
    }
  
  private:
    unsigned short* buffer;
    int idx;
    int buffer_len;
    unsigned short calculateAverage() {
      long sum = 0;
      for (int i=0; i<this->buffer_len; i++) {
        sum += buffer[i];
      }
      return sum / this->buffer_len;
    }
};


class OneEuroFilter {
  public:
    OneEuroFilter(float minCutoff, float beta, float dCutoff) {
      this->minCutoff = minCutoff;
      this->beta = beta;
      this->dCutoff = dCutoff;
      this->prevFilteredValue = 0.0;
      this->prevTimestamp = 0;
      this->initialized = false;
    }

    unsigned short filter(unsigned short value, unsigned long timestamp) {
      if (!initialized) {
        prevFilteredValue = value;
        prevTimestamp = timestamp;
        initialized = true;
        return value;
      }
      float dt = (float)(timestamp - prevTimestamp) / 1000.0; // Convert milliseconds to seconds
      float speed = abs(value - prevFilteredValue) / dt;
      float cutoff = minCutoff + beta * speed;
      float alpha = 1.0 / (1.0 + 2.0 * M_PI * cutoff * dt);
      float filteredValue = alpha * prevFilteredValue + (1.0 - alpha) * value;
      prevFilteredValue = filteredValue;
      prevTimestamp = timestamp;
      return filteredValue;
    }

  private:
    float minCutoff;
    float beta;
    float dCutoff;
    float prevFilteredValue;
    unsigned long prevTimestamp;
    bool initialized;
};

class KalmanFilter {
  public:
    KalmanFilter(float processNoise, float measurementNoise, float estimationError) {
      Q = processNoise;
      R = measurementNoise;
      P = estimationError;
      K = 0;
      prevEstimate = 0;
    }

    float filter(unsigned short value) {
      P += Q; // Update estimate
      K = P / (P + R);  // Calculate Kalman gain
      float estimate = prevEstimate + K * (value - prevEstimate); // Update the estimation
      P *= (1 - K); // Update error covariance
      prevEstimate = estimate;  // Update previous estimate
      return estimate;
    }

  private:
    float Q;
    float R;
    float P;
    float K;
    float prevEstimate;
};


/**************************************************
 * Helpers
 *************************************************/

// read analog input
// joystick uses 100k pots which isn't ideal... 
// the previous read analog value affects the current read value.
// By performing a dummy read of an unchanging pin DUMMY, it "flushes" the ADC
short axisRead(uint8_t pin) {
  analogRead(DUMMY); //dummy read
  return analogRead(pin);
}

// deadzone calculations - remap value to a new range
float remap(short val, short in_min, short in_max, short out_min, short out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
short centreDeadzone(short input, float centre_deadzone, short min_val, short max_val) {
  short half_deadzone = (max_val + -min_val) * (centre_deadzone / 2);
  if (input <= half_deadzone && input >= -half_deadzone) {
    return 0;
  } else {
    if (input > half_deadzone) {
      return remap(input, half_deadzone, max_val, 0, max_val);
    } else {
      return remap(input, min_val, -half_deadzone, min_val, 0);
    }
  }
}

// print value to serial with fixed format for ease of reading
void debugVal(const char* label, short value, bool newline=false)
{
  Serial.print(label);
  Serial.print(": ");
  Serial.printf("%+05d", value);
  if (!newline) {
    Serial.print(", ");
  } else {
    Serial.println();
  }
}

// home led
void led(unsigned char lednum, unsigned char ledval)
{
  if (lednum==LED_GREEN) //control green and turn off red
  {
    digitalWrite(LED_GREEN, !ledval); //LED is active low
    digitalWrite(LED_RED, 1); //LED is active low
  }
  else if (lednum==LED_RED) //control red and turn off green
  {
    digitalWrite(LED_GREEN, 1); //LED is active low
    digitalWrite(LED_RED, !ledval); //LED is active low
  }
  else if (lednum==LED_ORANGE) //control green and red
  {
    digitalWrite(LED_GREEN, !ledval); //LED is active low
    digitalWrite(LED_RED, !ledval); //LED is active low
  }
  else //LED_PRESET
    digitalWrite(LED_PRESET, ledval); //LED is active high
}


/**************************************************
 * State
 *************************************************/

// button to pin mapping
unsigned char btn_pin[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, MAPPING_BTN, PRESET_BTN, BACK_SWITCH};
// arrays holding analog axis values are all indexed as follows;
// 0=x, 1=y, 2=z, 3=rx, 4=ry, 5=rz, 6=s0, 7=s1
const char* axis_labels[] = {"X", "Y", "Z", "RX", "RY", "RZ", "S0", "S1"};
int axis_pins[] = {X_AXIS, Y_AXIS, Z_AXIS, RX_AXIS, RY_AXIS, RZ_AXIS, S0_AXIS, S1_AXIS};
// start axis values, assumes when plugged in, all axes centered, except Z is at 100% (TODO: centre-zero mode calibration)
unsigned short start[NUM_AXES];
// raw samples
unsigned short raw[NUM_AXES];
// filtered values sent to the joystick controller class
short send[NUM_AXES];
// filters 
SimpleMovingAverage* SMA[NUM_AXES];
OneEuroFilter* OEF[NUM_AXES];
KalmanFilter* KF[NUM_AXES];


/**************************************************
 * Run
 *************************************************/

void setup()
{
#ifdef SERIAL_ENABLED
  // serial port for debugging
  Serial.begin(9600);
#endif

  if (SERIAL_DEBUG || SERIAL_DEBUG_LOOP) {
    while (!Serial) {
      // wait for Serial Monitor to be ready
    }
  }

  // read analog inputs with 12-bit resolution
  analogReadResolution(12);

  // set up joystick data, then manually send all at once
  HotasX.useManualSend(true);

  // set digital inputs
  for (int i=0; i<NUM_BUTTONS; i++) {
    pinMode(btn_pin[i], INPUT_PULLUP);
  }
  pinMode(HU, INPUT_PULLUP);
  pinMode(HD, INPUT_PULLUP);
  pinMode(HL, INPUT_PULLUP);
  pinMode(HR, INPUT_PULLUP);

  // set LED outputs
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, 1); //LED is active low
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, 1); //LED is active low
  pinMode(LED_PRESET, OUTPUT);
  digitalWrite(LED_PRESET, 0); //LED is active high

  // init filters
  for (int i=0; i<NUM_AXES; i++) {
    SMA[i] = new SimpleMovingAverage(SMA_BUF_LEN);
    OEF[i] = new OneEuroFilter(OEF_MIN_CUTOFF, OEF_BETA, OEF_D_CUTOFF);
    KF[i] = new KalmanFilter(KF_PROCESS_NOISE, KF_MEASUREMENT_NOISE, KF_ESTIMATION_ERROR);
  }

  // calibrate analog inputs
  // get an avg of the inputs for initial values (using the SMA filter)
  for (int i=0; i<SMA_BUF_LEN; i++) {
    start[0] = SMA[0]->filter(axisRead(axis_pins[0]));
    start[1] = SMA[1]->filter(axisRead(axis_pins[1]));
    if (INV_Z == 0) {
      start[2] = SMA[2]->filter(axisRead(axis_pins[2])) - MAX_AVAL;
    } else {
      start[2] = SMA[3]->filter(axisRead(axis_pins[2])) + MAX_AVAL;
    }
    start[5] = SMA[5]->filter(axisRead(axis_pins[5]));
    start[6] = SMA[6]->filter(axisRead(axis_pins[6]));
    if (ENABLE_ALL_AXIS) {
      start[3] = SMA[3]->filter(axisRead(axis_pins[3]));
      start[4] = SMA[4]->filter(axisRead(axis_pins[4]));
      start[7] = SMA[7]->filter(axisRead(axis_pins[7]));
    } else {
      start[3] = MIN_AVAL;
      start[4] = MIN_AVAL;
      start[7] = 0;
    }
  }

  if (SERIAL_DEBUG) {
    Serial.println("Power-on calibration of zero points...");
    debugVal("Start X", start[0], 1);
    debugVal("Start Y", start[1], 1);
    debugVal("Z calibration value (max)", SMA[2]->currentAverage(), 1); // throttle assumes 100% for calibration
    debugVal("Start Z (the calculated centre point)", start[2], 1);  // assumes MAX_AVAL is accurate representation
    debugVal("Start RZ", start[5], 1);
    debugVal("Start S0", start[6], 1);
    if (ENABLE_ALL_AXIS) {
      debugVal("Start RX", start[3], 1);
      debugVal("Start RY", start[4], 1);
      debugVal("Start S1", start[5], 1);
    }
    Serial.print("Using FILTER_TYPE ");
    Serial.println(FILTER_TYPE);
  }
}

void loop()
{
  int i;

  // sample analog hardware
  for (i=0; i<NUM_AXES; i++) {
    raw[i] = axisRead(axis_pins[i]);
  }
  if (!ENABLE_ALL_AXIS) {
    raw[3] = MIN_AVAL;
    raw[4] = MIN_AVAL;
    raw[7] = 0;
  }

  // apply filtering
  unsigned long timestamp = millis();
  for (i=0; i<NUM_AXES; i++) {
    switch(FILTER_TYPE) {
      case 0:
        send[i] = raw[i]; // no filter
        break;
      case 1:
        send[i] = SMA[i]->filter(raw[i]); // simple moving average
        break;
      case 2:
        send[i] = OEF[i]->filter(raw[i], timestamp);  // one euro filter
        break;
      case 3:
        send[i] = KF[i]->filter(raw[i]);  // kalman filter
        break;
    }
  }

  // apply centre point calibration
  for (i=0; i<NUM_AXES; i++) {
    send[i] = send[i] - start[i];
  }

  // apply deadzones
  send[0] = centreDeadzone(send[0], X_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);
  send[1] = centreDeadzone(send[1], Y_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);
  send[2] = send[2] * (1 + THROTTLE_LIMIT_DEADZONE);  // just increase scale by deadzone %
  send[3] = centreDeadzone(send[3], RX_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);
  send[4] = centreDeadzone(send[4], RY_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);
  send[5] = centreDeadzone(send[5], RZ_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);
  send[6] = centreDeadzone(send[6], S0_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);
  send[7] = centreDeadzone(send[7], S1_CENTRE_DEADZONE, MIN_AVAL, MAX_AVAL);

  // send final values to joystick controller class
  HotasX.X(INV_X, send[0]);
  HotasX.Y(INV_Y, send[1]);
  HotasX.Z(INV_Z, send[2]);
  HotasX.Xrotate(INV_RX, send[3]);
  HotasX.Yrotate(INV_RY, send[4]);
  HotasX.Zrotate(INV_RZ, send[5]);
  HotasX.slider(INV_S0, 0, send[6]);
  HotasX.slider(INV_S1, 1, send[7]);
  // read digital pins for the buttons
  for (int i=0; i<NUM_BUTTONS; i++) {
    HotasX.button(i + 1, !digitalRead(btn_pin[i]));
  }
  // read digital pins for the hat
  HotasX.hat(digitalRead(HU), digitalRead(HD), digitalRead(HL), digitalRead(HR));
  // done reading joystick data, now send
  HotasX.send_now();

  // switch throttle LED mode & update LED outputs
  digitalWrite(LED_PRESET, digitalRead(BACK_SWITCH) ? LOW : HIGH); // turn on if PC/PSP switch is set to PS
  if ( send[2] > MAX_AVAL ) {
    led(LED_GREEN, HIGH);
  } else if ( send[2] < MIN_AVAL ) {
    led(LED_RED, HIGH);
  } else if ( !digitalRead(BACK_SWITCH) && send[2] > (0 - (MAX_AVAL * (THROTTLE_MID_RANGE/2))) && send[2] < (0 + (MAX_AVAL * (THROTTLE_MID_RANGE/2))) ) {
    led(LED_ORANGE, HIGH);
  } else {
    led(LED_GREEN, LOW);
  }

  // debug
  char raw_label[8];
  char send_label[8];
  if (SERIAL_DEBUG_LOOP) {
    for (int i=0; i<NUM_AXES; i++) {
      sprintf(raw_label, "Raw %s", axis_labels[i]);
      sprintf(send_label, "Send %s", axis_labels[i]);
      debugVal(raw_label, raw[i]);
      if (i < NUM_AXES - 1) {
        debugVal(send_label, send[i]);
      } else {
        debugVal(send_label, send[i], 1);
      }
    }
  }

  // throttle the joystick refresh rate
  delay(LOOP_DELAY);

}
