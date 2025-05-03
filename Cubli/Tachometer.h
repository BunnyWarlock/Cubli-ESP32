// Library to calculate angular speed from 12-bit magnetic encoder readings like AS5600

#ifndef Tachometer_h
#define Tachometer_h

class Tachometer{
  public:
    double prev_pos;
    double delta_raw;
    double delta_rev;
    double omega; // [rev/s]

    Tachometer();
    void setPos(double pos);
    double calcSpeed(double cur_pos, double deltat); // Returns rev/s
};

#endif