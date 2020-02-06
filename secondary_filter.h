#include "arduino.h"

class secondary_filter
{
private:
    double int_time;
    bool set_t;
    double preOutput;
    double prev_output1, prev_output2;
    bool set_secorder;

public:
    double T_LPF;
    secondary_filter(double);
    void setLowPassPara(double T, double init_data);
    double SecondOrderLag(double input);
    void setSecondOrderPara(double xOmega, double xDzeta, double init_data);

    double omega;
    double dzeta;
};
