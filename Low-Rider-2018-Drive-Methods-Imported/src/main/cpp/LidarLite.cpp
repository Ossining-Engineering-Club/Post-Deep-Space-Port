#include "LidarLite.h"

LidarLite::LidarLite(int dioPort):
    lidarCounter(dioPort)
{
    distance = 0.0;
    avgDistance = 0.0;
    lidarCounter.SetMaxPeriod(1.0);
    lidarCounter.SetSemiPeriodMode(true);
    lidarCounter.Reset();

}

double LidarLite::GetDistance(){
    if(lidarCounter.Get() < 1)
        distance = 0.0;
    else{
        distance = lidarCounter.GetPeriod()*100000.0 + LIDAR_OFFSET;
        lidarCounter.Reset();
        }
        avgDistance*=0.9;
        avgDistance += 0.1*distance;
    return avgDistance;
}
