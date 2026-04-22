/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include "DeviceDriverSet.h"
#include "util.h"

// ---------------------------------------------------------------------------
// RGB LED
// ---------------------------------------------------------------------------

// Packs individual R, G, B bytes into a single 32-bit colour word.
static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour) {
  if (NUM_LEDS < Traversal_Number) {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++) {
    leds[Number] = colour;
    FastLED.show();
    delay_wdt(Duration);
  }
}

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness) {
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(set_Brightness);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Test(void) {
  leds[0] = CRGB::White;
  FastLED.show();
  delay_wdt(50);
  leds[1] = CRGB::Red;
  FastLED.show();
  delay_wdt(50);
  DeviceDriverSet_RBGLED_xxx(50 /*Duration*/, 5 /*Traversal_Number*/, CRGB::Black);
}
#endif

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b) {
  if (LED_s > NUM_LEDS)
    return;
  if (LED_s == NUM_LEDS) {
    FastLED.showColor(Color(r, g, b));
  } else {
    leds[LED_s] = Color(r, g, b);
  }
  FastLED.show();
}

// ---------------------------------------------------------------------------
// Key Detection
// ---------------------------------------------------------------------------

uint8_t DeviceDriverSet_Key::keyValue = 0;

// ISR: each falling edge on PIN_Key advances the mode counter with 500 ms debounce.
static void attachPinChangeInterrupt_GetKeyValue(void) {
  DeviceDriverSet_Key Key;
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;
  if ((millis() - keyValue_time) > 500) {
    keyValue_temp++;
    keyValue_time = millis();
    if (keyValue_temp > keyValue_Max) {
      keyValue_temp = 0;
    }
    Key.keyValue = keyValue_temp;
  }
}

void DeviceDriverSet_Key::DeviceDriverSet_Key_Init(void) {
  pinMode(PIN_Key, INPUT_PULLUP);
  attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Key::DeviceDriverSet_Key_Test(void) {
  Serial.println(DeviceDriverSet_Key::keyValue);
}
#endif

void DeviceDriverSet_Key::DeviceDriverSet_key_Get(uint8_t *get_keyValue) {
  *get_keyValue = keyValue;
}

// ---------------------------------------------------------------------------
// ITR20001 Line-tracking IR Sensors
// ---------------------------------------------------------------------------

bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void) {
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}

int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void) {
  return analogRead(PIN_ITR20001xxxL);
}

int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void) {
  return analogRead(PIN_ITR20001xxxM);
}

int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void) {
  return analogRead(PIN_ITR20001xxxR);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Test(void) {
  Serial.print("\tL=");
  Serial.print(analogRead(PIN_ITR20001xxxL));
  Serial.print("\tM=");
  Serial.print(analogRead(PIN_ITR20001xxxM));
  Serial.print("\tR=");
  Serial.println(analogRead(PIN_ITR20001xxxR));
}
#endif

// ---------------------------------------------------------------------------
// Voltage Detection
// ---------------------------------------------------------------------------

void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Init(void) {
  pinMode(PIN_Voltage, INPUT);
}

float DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_getAnalogue(void) {
  // 0.0375 = (5 V / 1024) * voltage-divider ratio (10 kΩ + 1.5 kΩ) / 1.5 kΩ
  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  // +8% empirical compensation for component tolerances
  Voltage = Voltage + (Voltage * 0.08);
  return Voltage;
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Test(void) {
  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08);
  Serial.println(Voltage);
}
#endif

// ---------------------------------------------------------------------------
// Motor (TB6612 dual H-bridge)
// ---------------------------------------------------------------------------

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void) {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void) {
  // A = right motor, B = left motor
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);
  delay_wdt(1000);

  digitalWrite(PIN_Motor_STBY, LOW);
  delay_wdt(1000);

  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);
  delay_wdt(1000);

  // Stop motors before returning.
  digitalWrite(PIN_Motor_STBY, LOW);
}
#endif

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A,
                                                          boolean direction_B, uint8_t speed_B,
                                                          boolean controlED) {
  if (controlED == control_enable) {
    digitalWrite(PIN_Motor_STBY, HIGH);

    // Group A — right motor
    switch (direction_A) {
      case DIR_FORWARD:
        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case DIR_BACKWARD:
        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case DIR_STOP:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
    }

    // Group B — left motor
    switch (direction_B) {
      case DIR_FORWARD:
        digitalWrite(PIN_Motor_BIN_1, HIGH);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case DIR_BACKWARD:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case DIR_STOP:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
    }
  } else {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}

// ---------------------------------------------------------------------------
// Ultrasonic Sensor (HC-SR04)
// ---------------------------------------------------------------------------

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init(void) {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/) {
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // pulseIn returns µs; divide by 58 to convert to cm (speed of sound round-trip: ~58 µs/cm).
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);

  if (tempda_x > 150) {
    *ULTRASONIC_Get = 150;
  } else {
    *ULTRASONIC_Get = tempda_x;
  }
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Test(void) {
  unsigned int tempda = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  Serial.print("ULTRASONIC=");
  Serial.print(tempda);
  Serial.println("cm");
}
#endif

// ---------------------------------------------------------------------------
// Servo
// ---------------------------------------------------------------------------

// 0.17 s / 60° at 4.8 V — MOVE_TIME_MS is set conservatively to cover full travel.
Servo myservo;

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle) {
  // 500 µs = 0°, 2400 µs = 180°
  myservo.attach(PIN_Servo_z, 500, 2400);
  myservo.write(Position_angle);
  delay_wdt(500);

  myservo.attach(PIN_Servo_y, 500, 2400);
  myservo.write(Position_angle);
  delay_wdt(500);
  myservo.detach();
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Test(void) {
  myservo.attach(PIN_Servo_z);
  myservo.write(180);
  delay_wdt(500);
  myservo.write(0);
  delay_wdt(500);
  myservo.write(90);
  delay_wdt(500);
  myservo.detach();
}
#endif

// Non-blocking: attach, write, record start time; Tick() detaches after MOVE_TIME_MS.
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_SetAngle(unsigned int Position_angle) {
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
  _moveStart = millis();
  _moving = true;
}

// Non-blocking: attach, write, record start time; Tick() detaches after MOVE_TIME_MS.
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_controls(uint8_t Servo, unsigned int Position_angle) {
  if (Servo == 1 || Servo == 3) { // Servo_z
    if (Position_angle <= 1)      // minimum angle
      Position_angle = 1;
    if (Position_angle >= 17)     // maximum angle
      Position_angle = 17;
    myservo.attach(PIN_Servo_z);
    myservo.write(10 * Position_angle);
  }
  if (Servo == 2 || Servo == 3) { // Servo_y
    if (Position_angle <= 3)      // minimum angle
      Position_angle = 3;
    if (Position_angle >= 11)     // maximum angle
      Position_angle = 11;
    myservo.attach(PIN_Servo_y);
    myservo.write(10 * Position_angle);
  }
  _moveStart = millis();
  _moving = true;
}

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Tick() {
  if (_moving && (millis() - _moveStart >= MOVE_TIME_MS)) {
    myservo.detach();
    _moving = false;
  }
}

bool DeviceDriverSet_Servo::DeviceDriverSet_Servo_IsReady() {
  return !_moving;
}

// ---------------------------------------------------------------------------
// IR Receiver
// ---------------------------------------------------------------------------

IRrecv irrecv(RECV_PIN);
decode_results results;

// IR lookup table: two remote control sets (A and B), 14 buttons each.
// buttonId mapping: up=1, down=2, left=3, right=4, ok=5, 1–9 = 6–14.
const DeviceDriverSet_IRrecv::IrEntry DeviceDriverSet_IRrecv::IR_TABLE[IR_TABLE_SIZE * 2] = {
  // Remote A codes
  { 16736925UL,  1 }, // upper
  { 16754775UL,  2 }, // lower
  { 16720605UL,  3 }, // left
  { 16761405UL,  4 }, // right
  { 16712445UL,  5 }, // ok
  { 16738455UL,  6 }, // 1
  { 16750695UL,  7 }, // 2
  { 16756815UL,  8 }, // 3
  { 16724175UL,  9 }, // 4
  { 16718055UL, 10 }, // 5
  { 16743045UL, 11 }, // 6
  { 16716015UL, 12 }, // 7
  { 16726215UL, 13 }, // 8
  { 16734885UL, 14 }, // 9
  // Remote B codes
  {    5316027UL,  1 }, // upper
  { 2747854299UL,  2 }, // lower
  { 1386468383UL,  3 }, // left
  {  553536955UL,  4 }, // right
  { 3622325019UL,  5 }, // ok
  { 3238126971UL,  6 }, // 1
  { 2538093563UL,  7 }, // 2
  { 4039382595UL,  8 }, // 3
  { 2534850111UL,  9 }, // 4
  { 1033561079UL, 10 }, // 5
  { 1635910171UL, 11 }, // 6
  { 2351064443UL, 12 }, // 7
  { 1217346747UL, 13 }, // 8
  {   71952287UL, 14 }, // 9
};

void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void) {
  // Enable NEC-protocol IR reception.
  irrecv.enableIRIn();
}

bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/) {
  if (irrecv.decode(&results)) {
    IR_PreMillis = millis();
    for (uint8_t i = 0; i < IR_TABLE_SIZE * 2; i++) {
      if (IR_TABLE[i].code == results.value) {
        *IRrecv_Get = IR_TABLE[i].buttonId;
        irrecv.resume();
        return true;
      }
    }
    // Unknown code — resume so the next frame can be received.
    irrecv.resume();
    return false;
  } else {
    return false;
  }
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Test(void) {
  if (irrecv.decode(&results)) {
    Serial.print("IRrecv_Test:");
    Serial.println(results.value);
    irrecv.resume();
  }
}
#endif
