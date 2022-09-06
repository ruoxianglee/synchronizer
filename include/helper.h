#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <chrono>
#include <random>

using namespace std;
using namespace chrono;

#define Mstos 1000
rclcpp::Duration PeriodBase = rclcpp::Duration(1, 0); //1s

double cal_time_disparity(int channel_num, double t1, double t2, double t3 = 0., double t4 = 0., double t5 = 0., double t6 = 0., double t7 = 0., double t8 = 0., double t9 = 0.)
{
    double max_td = t1;
    double min_td = t1;
    for(int i = 2; i <= channel_num; i++)
    {
        switch(i)
        {
        case 2:
            if(t2 < min_td)
                min_td = t2;
            if(t2 > max_td)
                max_td = t2;
            break;
        case 3:
            if(t3 < min_td)
                min_td = t3;
            if(t3 > max_td)
                max_td = t3;
            break;
        case 4:
            if(t4 < min_td)
                min_td = t4;
            if(t4 > max_td)
                max_td = t4;
            break;
        case 5:
            if(t5 < min_td)
                min_td = t5;
            if(t5 > max_td)
                max_td = t5;
            break;
        case 6:
            if(t6 < min_td)
                min_td = t6;
            if(t6 > max_td)
                max_td = t6;
            break;
        case 7:
            if(t7 < min_td)
                min_td = t7;
            if(t7 > max_td)
                max_td = t7;
            break;
        case 8:
            if(t8 < min_td)
                min_td = t8;
            if(t8 > max_td)
                max_td = t8;
            break;
        case 9:
            if(t9 < min_td)
                min_td = t9;
            if(t9 > max_td)
                max_td = t9;
            break;
        default:
            break;
        }
    }

    return (max_td - min_td) * Mstos; //ms 
}

// Message delay 1-40 ms
void delay_some_time(double & delay_previous, int period)
{
    int upper_ = 40; //ms
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);

    int delay_upper_now;
    if(delay_previous <= period)
    {
        delay_upper_now = upper_;
    }
    else
    {
        delay_upper_now = period + upper_ - delay_previous;
    }

    if(delay_previous > upper_ + period)
    {
        delay_upper_now = 0;
    }

    if(delay_previous > upper_)
    {
        delay_upper_now -= delay_previous - upper_;
    }

    uniform_int_distribution<unsigned> bias(1, delay_upper_now); // bias for T
    int delay_now = bias(gen);

    auto start = system_clock::now();

    if(delay_now >= 1 && delay_now <= delay_upper_now)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(delay_now));
    }

    auto end   = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    double observed_delay = double(duration.count()) * microseconds::period::num / microseconds::period::den * Mstos;

    if(delay_previous <= period)
    {
        delay_previous = observed_delay; // +=???
    }
    else
    {
        delay_previous += observed_delay - period;
    }
}

// Message delay 0-delay_upper_ ms
void delay_random_time(double & delay_previous, int period, int delay_upper_)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);
    bernoulli_distribution delay_flag; // delay or not

    if(delay_flag(gen))
    {
        int delay_upper_now;
        if(delay_previous <= period)
        {
            delay_upper_now = delay_upper_;
        }
        else
        {
            delay_upper_now = period + delay_upper_ - delay_previous;
        }

        if(delay_previous > delay_upper_ + period)
        {
            delay_upper_now = 0;
        }

        if(delay_previous > delay_upper_)
        {
            delay_upper_now -= delay_previous - delay_upper_;
        }

        uniform_int_distribution<unsigned> bias(0, delay_upper_now); // bias for T
        int delay_now = bias(gen);

        auto start = system_clock::now();
  
        if(delay_now >= 0 && delay_now <= delay_upper_now)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(delay_now));
        }

        auto end   = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        double observed_delay = double(duration.count()) * microseconds::period::num / microseconds::period::den * Mstos;

        if(delay_previous <= period)
        {
            delay_previous = observed_delay; // +=???
        }
        else
        {
            delay_previous += observed_delay - period;
        }
    }
    else
    {
        if(delay_previous <= period)
        {
            delay_previous = 0; // 0 ??? 
        }
        else
        {        
            delay_previous += 0 - period;
        }
    }
}
