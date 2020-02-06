class PID
{
private:
    bool factor_input_check;
    double preError;
    double intError;
    double Kp;
    double Ki;
    double Kd;
    double int_time;

public:
    PID(double, double, double, double);
    void PIDinit(double, double);
    double getCmd(double, double, double);
};
