//
// Created by Stefan Fabian on 14.06.18.
//

#ifndef HECTOR_PROFILING_TIMER_H
#define HECTOR_PROFILING_TIMER_H

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace hector_profiling
{

/*!
 * Time class that can be used for easy profiling.
 * The runtime of a single method can be measured using the static executeTime method.
 * To measure multiple runs use a Timer instance and pass true to the reset method between runs.
 */
class Timer
{
public:
  enum TimeUnit
  {
    Default = 0,
    Seconds = 1,
    Milliseconds = 2,
    Microseconds = 3,
    Nanoseconds = 4
  };

  template<typename T>
  struct TimerResult
  {
    long time;
    Timer::TimeUnit  time_unit;
    std::shared_ptr<T> result;

    std::string toString(const std::string &name)
    {
      std::vector<long> times;
      times.push_back(time);
      return Timer::internalPrint(name, times, time_unit, Timer::TimeUnit::Default);
    }
  };

  /*!
   * Constructs a new Timer instance.
   * @param clock_time_unit Sets the time unit the duration is measured in and returned by getElapsedTime().
   * @param print_time_unit The time unit used for printing. If Default the time unit is automatically chosen.
   */
  explicit Timer( std::string name, TimeUnit clock_time_unit = Milliseconds, TimeUnit print_time_unit = Default );

  void start();

  void stop();

  /*!
   * Resets the timer. If new_run is false all runs are cleared as well.
   * If you want to time multiple runs pass true.
   * @param new_run Whether or not you want to time a new run. If false, everything is reset including the runs.
   */
  void reset( bool new_run = false );

  long getElapsedTime() const;

  std::vector<long> getRunTimes() const;

  std::string toString() const;

  template<typename T>
  static std::unique_ptr<TimerResult<T>> time( const std::function<T(void)> &function, TimeUnit time_unit = Milliseconds );

protected:
  static std::string internalPrint( const std::string &name, const std::vector<long> &run_times,
                                    TimeUnit clock_time_unit, TimeUnit print_time_unit );

  static long internalGetDuration( const std::chrono::high_resolution_clock::time_point &start,
                                   const std::chrono::high_resolution_clock::time_point &end,
                                   TimeUnit time_unit );

  std::string name_;
  bool running_;
  std::vector<long> run_times_;
  long elapsed_time_;
  std::chrono::high_resolution_clock::time_point start_;
  TimeUnit clock_time_unit_;
  TimeUnit print_time_unit_;
};

template<typename T>
std::unique_ptr<Timer::TimerResult<T>> Timer::time( const std::function<T(void)> &function, Timer::TimeUnit time_unit )
{
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  T function_result = function();
  long time = internalGetDuration( start, std::chrono::high_resolution_clock::now(), time_unit );
  std::unique_ptr<Timer::TimerResult<T> > result(new Timer::TimerResult<T>());
  result->time = time;
  result->time_unit = time_unit;
  result->result.reset(new T(function_result));
  return result;
}

template <>
std::unique_ptr<Timer::TimerResult<void>> Timer::time<void>( const std::function<void(void)> &function, Timer::TimeUnit time_unit);

}

#endif //HECTOR_PROFILING_TIMER_H
