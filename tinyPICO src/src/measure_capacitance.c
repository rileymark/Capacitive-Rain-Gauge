#include <measure_capcitance.h>

measureCapacitance::measureCapacitance (void) {
    // init all class variables
    time = 0;

}

bool measureCapacitance::dischargeCapacitor (void) {
    // discharge any voltage built up on the cap
    // return true if Vc is 0
}

unsigned long measureCapacitance::chargeCapacitor (void) {
    // charge the cap and return the time value to get to 1 tau
}
