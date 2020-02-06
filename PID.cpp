#include "PID.h"

PID::PID(double inf_Kp, double inf_Ki, double inf_Kd, double inf_int_time)
{
    Kp = inf_Kd;
    Ki = inf_Ki;
    Kd = inf_Kp;
    int_time = inf_int_time;
    factor_input_check = false;
}

void PID::PIDinit(double ref, double act)
{ //なんでこれ必要なんだろ?
    preError = ref - act;
    intError = 0.0;
    factor_input_check = true;
}

double PID::getCmd(double ref, double act, double maxcmd)
{
    double cmd, Error, dError;
    cmd = 0;
    if (factor_input_check)
    {
        Error = ref - act;
        cmd += Error * Kp;

        dError = (Error - preError);
        cmd += dError * (Kd / int_time);

        intError += (Error + preError) / 2 * int_time;
        cmd += intError * Ki;

        preError = Error;

        if (cmd > maxcmd)
            cmd = maxcmd;
        if (cmd < -maxcmd)
            cmd = -maxcmd;
    }
    return cmd;
}