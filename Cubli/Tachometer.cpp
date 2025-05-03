#include "Tachometer.h"

Tachometer::Tachometer(){}

void Tachometer::setPos(double pos){
  prev_pos = pos;
}

double Tachometer::calcSpeed(double cur_pos, double deltat){
  // Handle wrap-around (encoder jumps from 4095 to 0 or vice-versa)
  delta_raw = cur_pos - prev_pos;
  prev_pos = cur_pos;
  if (delta_raw > 2048)
    delta_raw -= 4096;
  else if (delta_raw < -2048)
    delta_raw += 4096;

  delta_rev = delta_raw / 4096.0;
  omega = delta_rev / deltat; // [rev/s]
  return omega;
}