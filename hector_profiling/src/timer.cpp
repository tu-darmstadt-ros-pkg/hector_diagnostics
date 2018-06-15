//
// Created by Stefan Fabian on 14.06.18.
//

#include "hector_profiling/timer.h"

#include <sstream>
#include <iostream>
#include <cmath>

namespace hector_profiling
{
//std::string internalPrint( const std::string &name, const std::vector<long> &run_times,
//                           Timer::TimeUnit clock_time_unit, Timer::TimeUnit print_time_unit );
//
//long internalGetDuration( const std::chrono::high_resolution_clock::time_point &start,
//                          const std::chrono::high_resolution_clock::time_point &end,
//                          Timer::TimeUnit time_unit );

Timer::Timer( std::string name, TimeUnit clock_time_unit, TimeUnit print_time_unit )
  : elapsed_time_( 0 ), running_( false ), name_( std::move( name )), clock_time_unit_( clock_time_unit )
    , print_time_unit_( print_time_unit )
{
  if ( clock_time_unit_ == Default )
  {
    std::cout << "Invalid argument to timer! Clock time unit can not be 'Default'. Using Milliseconds." << std::endl;
    clock_time_unit_ = Milliseconds;
  }
  if ((int) clock_time_unit_ < (int) print_time_unit_ )
  {
    std::cout << "Invalid argument to timer! Clock time unit can not be greater than print time unit. "
                 "You can't measure in seconds and print in milliseconds. Print time unit set to Default." << std::endl;
    print_time_unit_ = Default;
  }
}

void Timer::start()
{
  if ( running_ ) return;
  start_ = std::chrono::high_resolution_clock::now();
  running_ = true;
}

void Timer::stop()
{
  if ( !running_ ) return;
  elapsed_time_ += internalGetDuration( start_, std::chrono::high_resolution_clock::now(), clock_time_unit_ );
  running_ = false;
}

void Timer::reset(bool new_run)
{
  if (running_) stop();
  if (new_run)
  {
    run_times_.push_back(elapsed_time_);
  }
  else
  {
    run_times_.clear();
  }
  elapsed_time_ = 0;
}

long Timer::getElapsedTime() const
{
  long result = elapsed_time_;
  if ( running_ )
    result += internalGetDuration( start_, std::chrono::high_resolution_clock::now(), clock_time_unit_ );
  return elapsed_time_;
}

std::vector<long> Timer::getRunTimes() const
{
  std::vector<long> result = run_times_;
  if (elapsed_time_ != 0)
  {
    result.push_back(elapsed_time_);
  }
  return result;
}


template <>
std::unique_ptr<Timer::TimerResult<void>> Timer::time<void>( const std::function<void(void)> &function, Timer::TimeUnit time_unit)
{
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  function();
  long time = internalGetDuration( start, std::chrono::high_resolution_clock::now(), time_unit );
  std::unique_ptr<Timer::TimerResult<void> > result( new Timer::TimerResult<void>());
  result->time = time;
  return result;
}

std::string Timer::toString() const
{
  return internalPrint(name_, getRunTimes(), clock_time_unit_, print_time_unit_);
}

long Timer::internalGetDuration( const std::chrono::high_resolution_clock::time_point &start,
                                        const std::chrono::high_resolution_clock::time_point &end,
                                        TimeUnit time_unit )
{
  long result;
  auto diff = end - start;
  switch ( time_unit )
  {
    case Nanoseconds:
      result = std::chrono::duration_cast<std::chrono::nanoseconds>( diff ).count();
      break;
    case Microseconds:
      result = std::chrono::duration_cast<std::chrono::microseconds>( diff ).count();
      break;
    case Seconds:
      result = std::chrono::duration_cast<std::chrono::seconds>( diff ).count();
      break;
    case Default:
    case Milliseconds:
    default:
      result = std::chrono::duration_cast<std::chrono::milliseconds>( diff ).count();
  }
  return result;
}

template<typename T>
void printTimeString(std::stringstream &stream, T time, Timer::TimeUnit clock_time_unit,
                     Timer::TimeUnit print_time_unit)
{
  switch ( print_time_unit )
  {
    case Timer::Seconds:
      switch ( clock_time_unit )
      {
        case Timer::Seconds:
          stream << time << " seconds";
          break;
        case Timer::Microseconds:
          stream << time / 1E6 << " seconds";
          break;
        case Timer::Nanoseconds:
          stream << time / 1E9 << " seconds";
          break;
        case Timer::Milliseconds:
        default:
          stream << time / 1000.0 << " seconds";
          break;
      }
      break;
    case Timer::Milliseconds:
      switch ( clock_time_unit )
      {
        case Timer::Microseconds:
          stream << time / 1000.0 << " ms";
          break;
        case Timer::Nanoseconds:
          stream << time / 1E6 << " ms";
          break;
        case Timer::Milliseconds:
        default:
          stream << time << " ms";
          break;
      }
      break;
    case Timer::Microseconds:
      switch ( clock_time_unit )
      {
        case Timer::Nanoseconds:
          stream << time / 1000.0 << " us";
          break;
        case Timer::Microseconds:
        default:
          stream << time << " us";
          break;
      }
      break;
    case Timer::Nanoseconds:
      stream << time << " ns";
      break;
    case Timer::Default:
    default:
      switch ( clock_time_unit )
      {
        case Timer::Seconds:
          stream << time << " seconds";
          break;
        case Timer::Microseconds:
          if ( time < 5000 )
          {
            stream << time << " us";
          }
          else if ( time < 5E6 )
          {
            stream << time / 1E3 << " ms";
          }
          else
          {
            stream << time / 1E6 << " seconds";
          }
          break;
        case Timer::Nanoseconds:
          if ( time < 5000 )
          {
            stream << time << " ns";
          }
          else if ( time < 5E6 )
          {
            stream << time / 1E3 << " us";
          }
          else if ( time < 5E9 )
          {
            stream << time / 1E6 << " ms";
          }
          else
          {
            stream << time / 1E9 << " seconds";
          }
          break;
        case Timer::Milliseconds:
        default:
          if ( time < 5000 )
          {
            stream << time << " ms";
          }
          else
          {
            stream << time / 1000.0 << " seconds";
          }
          break;
      }
  }
}

double square(double x) { return x * x; }

std::string Timer::internalPrint( const std::string &name, const std::vector<long> &run_times, TimeUnit clock_time_unit,
                                  TimeUnit print_time_unit )
{
  std::stringstream stringstream;
  stringstream << "[Timer: " << name << "] " << run_times.size() << " run(s) took: ";
  if (run_times.size() == 1)
  {
    printTimeString(stringstream, run_times[0], clock_time_unit, print_time_unit);
    stringstream << "." << std::endl;
  }
  else
  {
    double mean = 0;
    long max = 0;
    long min = INT64_MAX;
    for (long time : run_times)
    {
      mean += time;
      if (time > max) max = time;
      if (time < min) min = time;
    }
    mean /= run_times.size();
    double var = 0;
    for (long time : run_times)
    {
      var += square(time - mean);
    }
    stringstream << std::endl << "On average: ";
    printTimeString(stringstream, mean, clock_time_unit, print_time_unit);
    stringstream << " +- ";
    printTimeString(stringstream, sqrt(var), clock_time_unit, print_time_unit);
    stringstream << "." << std::endl << "Longest run: ";
    printTimeString(stringstream, max, clock_time_unit, print_time_unit);
    stringstream << "." << std::endl << "Quickest run: ";
    printTimeString(stringstream, min, clock_time_unit, print_time_unit);
    stringstream << "." << std::endl << std::endl;
  }
  return stringstream.str();
}
}
