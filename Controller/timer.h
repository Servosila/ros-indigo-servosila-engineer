#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

#include <unistd.h>     /* usleep() */
#include <sys/time.h>   /*gettimeofday() */

namespace control
{

typedef useconds_t usec_t;

class timer
{
private:
    timeval m_set_time;
    timeval m_interval;
public:
    timer (usec_t uinterval = 0)
        :   m_set_time(),
            m_interval()
    {
        //computing the interval
        configure(uinterval);
    }

    void configure(usec_t uinterval)
    {
        //computing the interval
        m_interval.tv_sec =  uinterval / 1000000;
        m_interval.tv_usec = uinterval % 1000000;
        //setting the timer
        restart();
    }

    usec_t get_interval() const
    {
        return m_interval.tv_sec * 1000000 + m_interval.tv_usec;
    }

    void restart()
    {
        //getting "now"
        timeval now;
        gettimeofday(&now, nullptr);
        //setting the timer
        timeradd(&m_interval, &now, &m_set_time);
    }

    bool check() const
    {
        //getting "now"
        timeval now;
        gettimeofday(&now, nullptr);
        //comparing and returning the result
        return check(now);
    }

    bool check (timeval& now) const
    {
        //comparing and returning the result
        return timercmp(&now, &m_set_time, >);//this is a macro
    }

    bool check_and_restart()
    {
        bool result;
        //checking if the time has come
        if(check())
        {
            restart(); //resetting the timer again
            result = true;
        }
        else result = false;
        //
        return result;
    }

    bool check_and_restart(timeval& now)
    {
        bool result;
        //checking if the time has come
        if(check(now))
        {
            restart(); //resetting the timer again
            result = true;
        }
        else result = false;
        //
        return result;
    }

    /*
        sleep from now till the set time
    */
    void sleep_and_restart()
    {
        //getting "now"
        timeval now;
        gettimeofday(&now, nullptr);
        //checking if the set time has been passed already
        if(timercmp(&now, &m_set_time, <=))
        {
            //sleep time in sec & microsec
            timeval sleep_time;
            //computing the difference
            timersub(&m_set_time, &now, &sleep_time);
            //converting into the microseconds
            const useconds_t usec = sleep_time.tv_sec * 1000000 + sleep_time.tv_usec;
            //
            //falling asleep
            ::usleep(usec);
        }
        //resetting the timer
        restart();
    }
}; //class timer

class stopwatch
{
private:
    timeval m_start_time;
public:
    stopwatch() : m_start_time()
    {
        restart();
    }

    void restart()
    {   //setting start time as "now"
        gettimeofday(&m_start_time, nullptr);
    } //restart()

    usec_t get_elapsed_usec() const
    {
        //getting "now"
        timeval now;
        gettimeofday(&now, nullptr);
        //sleep time in sec & microsec
        timeval elapsed_time;
        //computing the difference
        timersub(&now, &m_start_time, &elapsed_time);
        //converting into the microseconds
        const usec_t result_usec = elapsed_time.tv_sec * 1000000 + elapsed_time.tv_usec;
        //
        return result_usec;
    } //get_elapsed_time()
}; //class stopwatch

} //namespace control


#endif // TIMER_H_INCLUDED
