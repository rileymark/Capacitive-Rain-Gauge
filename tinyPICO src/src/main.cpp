#define _TIMERINTERRUPT_LOGLEVEL_     4
#include <Adafruit_ADS1X15.h>
#include <ESP32TimerInterrupt.h>
#include <soc/rtc_wdt.h>

Adafruit_ADS1115 ads;

// Pin connected to the ALERT/RDY signal for new sample notification.
constexpr int READY_PIN = 32;
constexpr int CHARGE_PIN = 15;
constexpr int DISCHARGE_PIN = 14;


#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define TIMER0_INTERVAL_US        1000

volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

enum cap_sm_states {CSM_start, CSM_charge, CSM_discharge, CSM_wait} cap_state;
float capVoltage = 0;
float voltageThreshold = 0.623 * 3.3;
bool tickFunctionFlag = false;
int resistance = 2000;
double capacitance = 0;
long previousTime = 0;

typedef struct State_Struct{
  unsigned long timeElapsed;
  unsigned long period;
  uint8_t state;
  uint8_t (*TickFct)(uint8_t);
}Tasks;

Tasks tasks[3];
ESP32Timer ITimer0(0);

void Capacitor_Tick_Fctn(void) {
  static unsigned long timePerTau;
  static bool flag;

  // actions
  switch (cap_state) {
    case CSM_start:
      timePerTau = 0;
      break;

    case CSM_wait:
      timePerTau = 0;
      break;

    case CSM_charge:
      //Serial.println("Charging");
      // make sure the discharge pin is not connected to the circuit
      pinMode(DISCHARGE_PIN, INPUT);
      // start charging the capacitor
      digitalWrite(CHARGE_PIN, HIGH);
      
      timePerTau += TIMER0_INTERVAL_US;
      flag = true;
      break;

    case CSM_discharge:
      //Serial.println("Discharging");
      digitalWrite(CHARGE_PIN, LOW);
      pinMode(DISCHARGE_PIN, OUTPUT);
      digitalWrite(DISCHARGE_PIN, LOW);

      if (flag) {
        capacitance = (timePerTau / static_cast<double>(resistance));
        Serial.println("The capacitance is: ");
        Serial.println(capacitance, 10);
        Serial.println(timePerTau);
        Serial.println(capVoltage);
        //delay(10000);
        flag = false;
      }
      else {
        timePerTau = 0;
      }
      break;

    default:
      break;

  }

  // transitions
  switch (cap_state) {

    case CSM_start:
      cap_state = CSM_wait;
      break;

    case CSM_wait:
      cap_state = CSM_charge;
      break;

    case CSM_charge:
      //Serial.println(capVoltage, 8);
      if (capVoltage >= voltageThreshold) {
        //delay(1000);
        timePerTau = micros() - previousTime;
        cap_state = CSM_discharge;
      }
      else {
        cap_state = CSM_charge;
      }
      break;

    case CSM_discharge:
      //Serial.println(capVoltage, 8);
      if (capVoltage <= 0.0005) {
        cap_state = CSM_charge;
        //Serial.println("Were here");
      }
      else {
        cap_state = CSM_discharge;
        previousTime = micros();
        //Serial.println("Else statement");
      }
      break; 

    default:
      break;
  }

  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

bool IRAM_ATTR TimerHandler0 (void * timerNo) {
  tickFunctionFlag = true;
	return true;
}

bool setupADC (void) {
  ads.begin();
  Serial.println("1");

  // Wait a second to see if it will work
  delay(10000);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  Serial.println("2");
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  Serial.println("3");
  pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);
  Serial.println("4");
  // Start continuous conversions.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/true);
    Serial.println("5");

  return true;
}

void setupMeasurementSM (void) {
  uint8_t ii = 0;
  tasks[ii].timeElapsed = 0;
  tasks[ii].period = 10;
  //tasks[ii].TickFct = &Capacitor_Tick_Fctn; // name the capacitor tick function this
  tasks[ii].state = CSM_start;

  // xTaskCreate(
  //   Capacitor_Tick_Fctn,        // task function
  //   "Capacitor_Tick_Fctn",      // task name
  //   1000,                       // stack size in bytes
  //   NULL,                       // pass paramters into the task
  //   1,                          // task priority
  //   NULL                        // task handle
  // );

  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_US, TimerHandler0)) {
      Serial.println ("Starting Interrupt");
  }
  else {
    Serial.println("Interrupt Failed");
    Serial.println("2");
    while(1) {}
  }

}

void setup (void) {
  //rtc_wdt_disable();
  //rtc_wdt_protect_off();

  noInterrupts();

  Serial.begin(115200);
  Serial.println("Hello!");
  pinMode(CHARGE_PIN, OUTPUT);
  //digitalWrite(15, HIGH);

  // setupADC();

  //setupMeasurementSM();

  interrupts();
  Serial.println("7");
}

void loop (void) {
  Serial.println("8");

  if (tickFunctionFlag) {
    Capacitor_Tick_Fctn();
    tickFunctionFlag = false;
  }

  if (!new_data) {
    return;
  }
  int16_t results = ads.getLastConversionResults();
  //Serial.println(ads.computeVolts(results));
  capVoltage = ads.computeVolts(results);
  new_data = false;

  Serial.println("capVoltage");
  //delay(1);
}
