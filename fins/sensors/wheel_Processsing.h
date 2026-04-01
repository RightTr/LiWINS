#ifndef WHEEL_PROCESSING_H
#define WHEEL_PROCESSING_H

class WheelProcess
{
public:
    WheelProcess();
    ~WheelProcess();

    void process_wheel_data(double timestamp, double wheel_speed, double wheel_angle);



}

#endif