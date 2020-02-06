#include "secondary_filter.h"

secondary_filter::secondary_filter(double xint_time)
{
    int_time = xint_time;
    set_t = false;
}

void secondary_filter::setLowPassPara(double T, double init_data)
{
    T_LPF = T;
    preOutput = init_data;
    set_t = true;
}

void secondary_filter::setSecondOrderPara(double xOmega, double xDzeta, double init_data)
{
    omega = xOmega;
    dzeta = xDzeta;
    prev_output2 = prev_output1 = init_data;
    set_secorder = true;
}

double secondary_filter::SecondOrderLag(double input)
{
    if (!set_secorder)
    {
        return input;
    }
    else
    {
        double output = (2.0 * prev_output1 * (1.0 + dzeta * omega * int_time) - prev_output2 + pow(omega, 2.0) * pow(int_time, 2.0) * input) / (1.0 + 2.0 * dzeta * omega * int_time + pow(omega, 2.0) * pow(int_time, 2.0));
        prev_output2 = prev_output1;
        prev_output1 = output;
        return output;
    }
}