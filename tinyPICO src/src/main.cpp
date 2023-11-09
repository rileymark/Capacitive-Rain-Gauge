#include <Arduino.h>
#include <tinyPICO.h>
#include <measure_capacitance.h>

#include <ESP32TimerInterrupt.h>
#include <Adafruit_ADS1X15.h>

#define READY_PIN     3;
#define CHARGE_PIN    13;
#define DISCHARGE_PIN 14;
#define INTERRUPT_TIME 10;

//UMS3 ums3;

ESP32Timer ITimer0(0);

const double Vcc = 3.300;
const uint16_t maxADC = 65535;

volatile bool new_data = false;

//////// this really could all go in its own header and C file tbh... better organization

// define the task scheduler struct
typedef struct State_Struct{
  unsigned long timeElapsed;
  unsigned long period;
  uint8_t state;
  uint8_t (*TickFct)uint8_t;
}Tasks;

// create an instance of the task scheduler that is 3 items long
Tasks tasks[3];

// define the enums for the SMs
enum cap_sm_states = {CSM_start, CSM_charge, CSM_discharge, CSM_wait} cap_state;
enum screen_sm_state = {SC_start, SC_update, SC_wait} sc_state;
int voltageThreshold = 0;
int capVoltage = 0;


void initTaskScheduler (void) {

  // capacitor SM
  ii = 0;
  tasks[ii].timeElapsed = 0;
  tasks[ii].period = 10;
  tasks[ii].TickFct = &Capacitor_Tick_Fctn; // name the capacitor tick function this
  tasks[ii].state = CSM_start;

  ++ii;

  // screen SM
  tasks[ii].timeElapsed = 0;
  tasks[ii].period = 20;
  tasks[ii].TickFct = &Screen_Tick_Fctn; // name the capacitor tick function this
  tasks[ii].state = SC_start;

  ++ii;

  // other
  tasks[ii].timeElapsed = 0;
  tasks[ii].period = 20;
  tasks[ii].TickFct = ; // something here so it doesnt throw an error
  tasks[ii].state = ; // same here
}

uint8_t Capacitor_Tick_Fctn(uint8_t state) {
  static unsigned long timePerTau;
  // actions
  switch (cap_state) {
    case CSM_start:
      timePerTau = 0;
      break;

    case CSM_wait;
      timePerTau = 0;
      break;

    case CSM_charge:
      // make sure the discharge pin is not connected to the circuit
      pinMode(DISCHARGE_PIN, INPUT);
      // start charging the capacitor
      digitalWrite(CHARGE_PIN, HIGH);

      timerPerTau += tasks[0].period;
      break;

    case CSM_discharge:
      digitalWrite(CHARGE_PIN, LOW);
      pinMode(DISCHARGE_PIN, OUTPUT);
      digitalWrite(DISCHARGE_PIN, LOW);
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
      if (capVoltage >= voltageThreshold) {
        cap_state = CSM_discharge;
        capVoltage = 0;
      }
      else {
        cap_state = CSM_Charge
      }
      break;

    case CSM_discharge:
      if (capVoltage <= 0) {
        cap_state = CSM_charge;
      }
      else {
        cap_state = CSM_discharge;
      }
      break; 

    default:
      break;
  }
}

/*------------------------------------------------------------------------------------*/
/*                                    ADC Data Flag                                   */
/*                                                                                    */
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

/*------------------------------------------------------------------------------------*/
/*                               Display Initialization                               */
/*                                                                                    */
void initDisplay(void) {

}

/*------------------------------------------------------------------------------------*/
/*                                 ADC Initialization                                 */
/*                                                                                    */
void initADC(void) {
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);
  // Start continuous conversions.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
}

/*------------------------------------------------------------------------------------*/
/*                                        Setup                                       */
/*                                                                                    */
void setup() {
  // aint no interrupts gonna stop us
  noInterrupts();

  // init the touchscreen
  initDisplay();
  
  // start a timer interrupt to run the SM
  ITimer0.attachInterruptInterval(INTERRUPT_TIME * 1000, timer0ISR);
 
  // init the ADC
  initADC();

  // init the task scheduler
  initTaskScheduler();

  // let em fly
  interrupts();
}

/*------------------------------------------------------------------------------------*/
/*                                        Loop                                        */
/*                                                                                    */
void loop() {
// start the timer and begin charging capicitor
// constantly run ADC
// if ADC voltage outputs over 62.3% of Vcc then capture the time and calculate C from tau = R * C

  // set nwe 
  if (newData == true) {
      // read the ADC
      newData = false;
  }
}

/*------------------------------------------------------------------------------------*/
/*                              Interrupt Service Routine                             */
/*                                                                                    */
void IRAM_ATTR timer0ISR(void) {
  for (ii = 0, ii > numTasks; ++ii) {
    if (tasks[ii].timeElapsed > tasks[ii].period) {
      tasks[ii].state = tasks[i].TickFct(tasks[i].state);
      tasks[ii].timeElapsed = 0;
    }
    tasks[ii].timeElapsed += INTERRUPT_TIME;

  }
}
