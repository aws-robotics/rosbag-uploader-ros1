/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <sys/stat.h>
#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#endif
#include <ctime>

#include <queue>
#include <string>
#include <vector>
#include <list>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>

#include "rosbag/bag.h"
#include "rosbag/stream.h"
#include "rosbag/macros.h"

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

class ROSBAG_DECL OutgoingMessage
{
public:
    OutgoingMessage(std::string _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, ros::Time _time);
    OutgoingMessage() = default;
    std::string                         topic;
    topic_tools::ShapeShifter::ConstPtr msg;
    boost::shared_ptr<ros::M_string>    connection_header;
    ros::Time                           time;
};

class ROSBAG_DECL OutgoingQueue
{
public:
    OutgoingQueue(std::string _filename, std::shared_ptr<std::queue<OutgoingMessage>> _queue, ros::Time _time);

    std::string                  filename;
    std::shared_ptr<std::queue<OutgoingMessage>> queue;
    ros::Time                    time;
};

struct ROSBAG_DECL RecorderOptions
{
    RecorderOptions() = default;

    bool            trigger {false};
    bool            record_all {false};
    bool            regex {false};
    bool            do_exclude {false};
    bool            quiet {false};
    bool            append_date {true};
    bool            snapshot {false};
    bool            verbose {false};
    bool            publish {false};
    rosbag::CompressionType compression {rosbag::compression::Uncompressed};
    std::string     prefix {""};
    std::string     name {""};
    boost::regex    exclude_regex {};
    uint32_t        buffer_size {1048576 * 256};
    uint32_t        chunk_size {1024 * 768};
    uint32_t        limit {0};
    bool            split {false};
    uint64_t        max_size {0};
    uint32_t        max_splits {0};
    ros::Duration   max_duration {-1.0};
    std::string     node {""};
    uint64_t min_space {1024 * 1024 * 1024};
    std::string min_space_str {"1G"};
    ros::TransportHints transport_hints;

    std::vector<std::string> topics;
};

class ROSBAG_DECL Recorder
{
public:
    explicit Recorder(RecorderOptions options);

    void DoTrigger();

    bool IsSubscribed(std::string const& topic) const;

    boost::shared_ptr<ros::Subscriber> Subscribe(ros::NodeHandle & nh, std::string const& topic);

    int Run();

private:
    void PrintUsage();

    void UpdateFilenames();
    void StartWriting();
    void StopWriting();

    bool CheckLogging();
    bool ScheduledCheckDisk();
    bool CheckDisk();

    void SnapshotTrigger(std_msgs::Empty::ConstPtr trigger);
    // void doQueue(topic_tools::ShapeShifter::ConstPtr msg, std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
    void DoQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                 std::string const& topic,
                 ros::Subscriber * subscriber,
                 const boost::shared_ptr<int>& count);
    void DoRecord();
    void CheckNumSplits();
    bool CheckSize();
    bool CheckDuration(const ros::Time& t);
    void DoRecordSnapshotter();
    void DoCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle);

    bool ShouldSubscribeToTopic(std::string const& topic, bool from_node = false);

    template<class T>
    static std::string TimeToStr(T ros_t);

private:
    RecorderOptions               options_;
    ros::NodeHandle               nh_;

    rosbag::Bag                           bag_;

    std::string                   target_filename_;
    std::string                   write_filename_;
    std::list<std::string>        current_files_;

    std::set<std::string>         currently_recording_;  //!< set of currenly recording topics
    std::vector<boost::shared_ptr<ros::Subscriber>> subscribers_;

    int                           exit_code_;            //!< eventual exit code

    boost::condition_variable_any queue_condition_;      //!< conditional variable for queue
    boost::mutex                  queue_mutex_;          //!< mutex for queue
    std::shared_ptr<std::queue<OutgoingMessage>>  queue_;                //!< queue for storing
    uint64_t                      queue_size_;           //!< queue size

    uint64_t                      split_count_;          //!< split count

    std::queue<OutgoingQueue>     queue_queue_;          //!< queue of queues to be used by the snapshot recorders

    ros::Time                     last_buffer_warn_;

    ros::Time                     start_time_;

    bool                          writing_enabled_;
    boost::mutex                  check_disk_mutex_;
    ros::WallTime                 check_disk_next_;
    ros::WallTime                 warn_next_;

    ros::Publisher                pub_begin_write_;
};

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws

#endif
