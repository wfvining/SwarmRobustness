#ifndef DATA_H
#define DATA_H

struct DATA
{
    float lwVel = 0;
    float rwVel = 0;
};

struct ObstSensors
{
    float distance;
    int sensorID;
    bool turnRight;
};

#endif // DATA_H

