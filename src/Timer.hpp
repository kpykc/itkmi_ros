#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <chrono>

class Timer
{
private:
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;
//    std::string name;

public:
    Timer()
    {
        begin = std::chrono::steady_clock::now();
        end = begin;
    }

    void reset()
    {
        begin = std::chrono::steady_clock::now();
        end = begin;
    }

    double measure()
    {
        end = std::chrono::steady_clock::now();

        return std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();
    }

};

#endif
