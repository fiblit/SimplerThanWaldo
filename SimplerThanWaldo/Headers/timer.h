#ifndef TIMER_H
#define TIMER_H
#include <string>
#include <vector>
#include <chrono>
#include <iostream>


//this should really be an actual stack, but I'll just manually control it so I know I'm ending the right layer
namespace timer {
    typedef std::chrono::high_resolution_clock Clock;
#ifdef _WINDOWS
    typedef std::chrono::steady_clock::time_point Time;
#else
    typedef std::chrono::system_clock::time_point Time;
#endif

    void init_layers(int amount);
    void start(int layer_i, std::string name);
    void stop(int layer_i);
}

#endif//TIMER_H
