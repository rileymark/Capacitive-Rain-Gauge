#include <Arduino.h>
#include <tinyPICO.h>


#include <ESP32TimerInterrupt.h>
#include <Adafruit_ADS1X15.h>

#define READY_PIN 3;

//UMS3 ums3;

ESP32Timer ITimer0(0);

const double Vcc = 3.300;
const uint16_t maxADC = 65535;

volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

void initDisplay(void) {

}

void setup() {

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  initDisplay();

  ITimer0.attachInterruptInterval(1000 * 1000, timer0ISR);

  pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);
  // Start continuous conversions.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

}

void loop() {
// start the timer and begin charging capicitor
// constantly run ADC
// if ADC voltage outputs over 62.3% of Vcc then capture the time and calculate C from tau = R * C

  ADC Make it work

  if (vIn == 0.632 * Vcc) {
    time = micros();
  }
}

void IRAM_ATTR timer0ISR(void) {
  for (ii = numTasks,)
}
