//=================================================================================================
// Copyright (c) 2014, Johannes Meyer and contributors, Technische Universität Darmstadt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of copyright holder nor the names of its contributors may be used to
//   endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_TIMING_H
#define HECTOR_TIMING_H

#include <hector_diagnostic_msgs/TimingInfo.h>

#include <map>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/time.h>

namespace hector_diagnostics {

using namespace hector_diagnostic_msgs;

class TimingSection;

/**
 * @brief TimingAggregator is the container for a hector_diagnostics_msgs::TimingInfo message.
 * It collects the timing results from all sections that have been executed.
 */
class TimingAggregator
{
public:
    /**
     * @brief Typedef for a boost::shared_ptr<TimingAggregator>
     */
    typedef boost::shared_ptr<TimingAggregator> shared_ptr;

    /**
     * @brief Return a shared_ptr to the global TimingAggregator instance.
     * @return a shared_ptr to the global TimingAggregator instance.
     */
    static shared_ptr Instance();

    /**
     * @brief Release the global TimingAggregator instance.
     */
    static void Release();

    /**
     * @brief The global TimingAggregator constructor.
     */
    TimingAggregator();

    /**
     * @brief Clear all sections from the message.
     * @note The section indices might change after this method has been called.
     */
    void clear();

    /**
     * @brief Reset all sections in the message.
     * This method resets the cpu_time and wall_time entries in all sections, but without deleting them.
     */
    void reset() const;

    /**
     * @brief Collect and return timing information of all section and resets their state
     *
     * Typical usage:
     *code{.cpp}
     * TimingAggregator timing_aggregator;
     * ros::Publisher timing_publisher = ...;
     * while(ros::ok()) {
     *     // do something and add sections
     *     timing_publisher.publish(timing_aggregator.update());
     * }
     *endcode
     *
     * @param stamp The time stamp written to the message header (optional). Defaults to ros::Time::now().
     * @return A TimingInfo message.
     *
     */
    TimingInfo update(const ros::Time &stamp = ros::Time()) const;

    /**
     * @brief Start runtime measurement of a section.
     *
     * Pass this instance to toc() to signal that the section has been finished.
     * A section will be automatically finished if the returned TimingInfo instance goes out of scope.
     *
     * A section can be started and stopped multiple times during an update period of the TimingAggregator.
     *
     * Typical usage:
     *code{.cpp}
     * TimingSection section(timing_aggregator.tic("processing_step"));
     * // ... run processing_step
     * section.toc();
     *endcode
     *
     * @param section The name of the section
     * @return A TimingSection instance.
     * @sa TimingSection
     */
    TimingSection tic(const std::string &section);

    /**
     * @brief Finish/stop runtime measurement of a section.
     *
     * This function will update the corresponding entry in the TimingInfo message.
     *
     * @param section An instance of TimingSection returned by tic().
     */
    void toc(TimingSection &section);

private:
    friend class TimingSection;
    friend class ros::serialization::Serializer<TimingAggregator>;
    std::size_t sectionIndex(const std::string &name);
    void rebuildSectionMap();

    mutable boost::mutex mutex_;
    boost::shared_ptr<TimingInfo> message_;
    std::map<std::string,std::size_t> section_map_;
};

/**
 * @brief A TimingSection instance is the "stopwatch" of hector_timing.
 */
class TimingSection
{
public:
    /**
     * @brief TimingSection constructor.
     *
     * Typical usage:
     *code{.cpp}
     * {
     *     TimingSection section(timing_aggregator, "processing_step");
     *     // ... run processing_step
     * }
     *endcode
     *
     * @param aggregator A reference to a TimingAggregator instance that publishes the results.
     * @param name The name of the section. Make sure to not use the same section name twice within one TimingAggregator.
     * @param autostart If true (the default), the section is started automatically upon construction.
     */
    TimingSection(TimingAggregator &aggregator, const std::string &name = std::string(), bool autostart = true);

    /**
     * @brief TimingSection constructor that uses the global TimingAggregator instance.
     *
     * Typical usage:
     *code{.cpp}
     * {
     *     TimingSection section("processing_step");
     *     // ... run processing_step
     * }
     *endcode
     *
     * @param name The name of the section. Make sure to not use the same section name twice within one TimingAggregator.
     * @param autostart If true (the default), the section is started automatically upon construction.
     */
    TimingSection(const std::string &name = std::string(), bool autostart = true);

    /**
     * @brief TimingSection destructor.
     * Automatically finishs the section if it was still running.
     */
    ~TimingSection();

    /**
     * @brief Start runtime measurement of this section.
     * A section can be started and stopped multiple times during an update period of the TimingAggregator.
     */
    void tic();

    /**
     * @brief Finish/stop runtime measurement of this section.
     */
    void toc();

    /**
     * @brief Returns a reference to the underlying Timing entry of this section holding the results.
     * @return a reference to the underlying Timing entry of this section holding the results.
     */
    Timing &operator*();

    /**
     * @brief Returns a const reference to the underlying Timing entry of this section holding the results.
     * @return a const reference to the underlying Timing entry of this section holding the results.
     */
    const Timing &operator*() const;

private:
    TimingAggregator &aggregator_;
    std::size_t index_;
    ros::Time tic_cpu_time_;
    ros::Time tic_wall_time_;
};

/**
 * @brief Get the total CPU time consumed by the calling thread.
 *
 * This is a wrapper around lock_gettime(CLOCK_THREAD_CPUTIME_ID, ...);
 *
 * @return The total CPU time consumed by the calling thread as ros::Time instance.
 */
ros::Time getCpuTime();

/**
 * @brief Get the wall time consumed by the calling thread.
 * This is not true wall time as represented by CLOCK_REALTIME, but a monotonic time in order to reliably measure wall durations.
 * Do not use as an absolute value.
 *
 * This is a wrapper around lock_gettime(CLOCK_MONOTONIC, ...);
 *
 * @return The current time of the monotonic clock as ros::Time instance.
 */
ros::Time getWallTime();

} // namespace hector_diagnostics

ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(hector_diagnostics::TimingAggregator,
                                  ros::message_traits::md5sum<hector_diagnostic_msgs::TimingInfo>(),
                                  ros::message_traits::datatype<hector_diagnostic_msgs::TimingInfo>(),
                                  ros::message_traits::definition<hector_diagnostic_msgs::TimingInfo>())

namespace ros {
namespace serialization {

template <>
class Serializer<hector_diagnostics::TimingAggregator>
{
public:
    typedef hector_diagnostics::TimingAggregator T;

    template<typename Stream, typename T>
    inline static void write(Stream& stream, const T& t)
    {
        ros::serialization::serialize(stream, t.update());
    }

    template<typename T>
    inline static uint32_t serializedLength(const T& t)
    {
        boost::mutex::scoped_lock lock(t.mutex_);
        return ros::serialization::serializationLength(*t.message_);
    }
};

} // namespace serialization
} // namespace ros

#endif // HECTOR_TIMING_H
