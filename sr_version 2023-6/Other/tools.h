//
// Created by eski on 6/29/22.
//

#ifndef SR_SDUST_TOOLS_H
#define SR_SDUST_TOOLS_H

#include <sys/time.h>
#include <iostream>
#include <typeinfo>

#define name2str(name) (#name)
#define changeDirection(x) (x = -1 * x)

void GetNowTime(long &time){
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    long milli = curTime.tv_usec;
    long second = curTime.tv_sec;
    time = second * 1000000 + milli;
}




#endif //SR_SDUST_TOOLS_H
