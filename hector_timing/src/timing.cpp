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

#include <hector_diagnostics/timing.h>
#include <time.h>

#include <ros/console.h>
#include <boost/lexical_cast.hpp>

namespace hector_diagnostics {

static TimingAggregator::shared_ptr g_time_aggregator_global_instance;

TimingAggregator::shared_ptr TimingAggregator::Instance()
{
    static shared_ptr the_instance;
    if (!g_time_aggregator_global_instance) g_time_aggregator_global_instance.reset(new TimingAggregator);
    return g_time_aggregator_global_instance;
}

void TimingAggregator::Release()
{
    g_time_aggregator_global_instance.reset();
}

TimingAggregator::TimingAggregator()
    : message_(new TimingInfo())
{}

void TimingAggregator::clear()
{
    boost::mutex::scoped_lock lock(mutex_);
    message_->header.stamp = ros::Time();
    message_->timing.clear();
    section_map_.clear();
}

void TimingAggregator::reset() const
{
    message_->header.stamp = ros::Time();
    for(TimingInfo::_timing_type::iterator it = message_->timing.begin(); it != message_->timing.end(); ++it) {
        it->cpu_time = ros::Duration();
        it->wall_time = ros::Duration();
    }
}

TimingInfo TimingAggregator::update(const ros::Time &stamp) const
{
    boost::mutex::scoped_lock lock(mutex_);

    // fill stamp and copy the TimingInfo message
    if (message_->header.stamp.isZero()) message_->header.stamp = (stamp.isZero() ? ros::Time::now() : stamp);
    TimingInfo copy = *message_;

    // ...then reset time entries
    reset();

    // ...and return the copy
    return copy;
}

std::size_t TimingAggregator::sectionIndex(const std::string &name)
{
    boost::mutex::scoped_lock lock(mutex_);

    if (section_map_.count(name)) return section_map_.at(name);
    std::size_t index = message_->timing.size();
    Timing &timing = *message_->timing.insert(message_->timing.end(), Timing());
    timing.section_name = !name.empty() ? name : "section" + boost::lexical_cast<std::string>(index);
    section_map_[timing.section_name] = index;
    return index;
}

TimingSection TimingAggregator::tic(const std::string &section)
{
    return TimingSection(*this, section, true);
}

void TimingAggregator::toc(TimingSection &section)
{
    section.toc();
}

TimingSection::TimingSection(TimingAggregator &aggregator, const std::string &name, bool autostart)
    : aggregator_(aggregator)
{
    index_ = aggregator_.sectionIndex(name);
    if (autostart) tic();
}

TimingSection::TimingSection(const std::string &name, bool autostart)
    : aggregator_(*TimingAggregator::Instance())
{
    index_ = aggregator_.sectionIndex(name);
    if (autostart) tic();
}

TimingSection::~TimingSection()
{
    toc();
}

Timing &TimingSection::operator*()
{
    return aggregator_.message_->timing[index_];
}

const Timing &TimingSection::operator*() const
{
    return aggregator_.message_->timing[index_];
}

void TimingSection::tic()
{
    ROS_DEBUG_STREAM_NAMED("timing", "tic: section[" << index_ << "] " << (**this).section_name);

    tic_cpu_time_  = getCpuTime();
    tic_wall_time_ = getWallTime();
}

void TimingSection::toc()
{
    boost::mutex::scoped_lock lock(aggregator_.mutex_);

    if (!tic_cpu_time_.isZero()) {
        ros::Duration toc_cpu_duration = getCpuTime() - tic_cpu_time_;
        (**this).cpu_time += toc_cpu_duration;
        (**this).cpu_time_accumulated += toc_cpu_duration;
    }
    if (!tic_wall_time_.isZero()) {
        ros::Duration toc_wall_duration = getWallTime() - tic_wall_time_;
        (**this).wall_time += toc_wall_duration;
        (**this).wall_time_accumulated += toc_wall_duration;
    }

    tic_cpu_time_  = ros::Time();
    tic_wall_time_ = ros::Time();

    ROS_DEBUG_STREAM_NAMED("timing", "toc: section[" << index_ << "] " << (**this).section_name << ": cpu time " << (**this).cpu_time << ", wall time " << (**this).wall_time);
}

ros::Time getCpuTime()
{
    ros::Time cpu_time;
    timespec t;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    cpu_time.sec  = t.tv_sec;
    cpu_time.nsec = t.tv_nsec;
    return cpu_time;
}

ros::Time getWallTime()
{
    ros::Time cpu_time;
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    cpu_time.sec  = t.tv_sec;
    cpu_time.nsec = t.tv_nsec;
    return cpu_time;
}

} // namespace hector_diagnostics
