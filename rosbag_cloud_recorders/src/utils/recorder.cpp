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

/* 
 * This source code is adapted from https://github.com/ros/ros_comm/tree/melodic-devel/tools/rosbag
 * from commit https://github.com/ros/ros_comm/commit/550608510089600f95f31c10cdbdc1e41087d9ab.
 * This copy was needed in order to resolve the issue in https://github.com/aws-robotics/rosbag-uploader-ros1/issues/75
 */



#include "rosbag_cloud_recorders/utils/recorder.h"

#include <sys/stat.h>
#include <boost/filesystem.hpp>
// Boost filesystem v3 is default in 1.46.0 and above
// Fallback to original posix code (*nix only) if this is not true
#if BOOST_FILESYSTEM_VERSION < 3
  #include <sys/statvfs.h>
#endif
#include <ctime>

#include <queue>
#include <sstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include "ros/network.h"
#include "ros/xmlrpc_manager.h"
#include "xmlrpcpp/XmlRpc.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Time;

namespace Aws
{
namespace Rosbag
{
namespace Utils
{
// OutgoingMessage

OutgoingMessage::OutgoingMessage(string _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, Time _time) :
    topic(std::move(_topic)), msg(std::move(_msg)), connection_header(std::move(_connection_header)), time(_time)
{
}

// OutgoingQueue

OutgoingQueue::OutgoingQueue(string _filename, std::shared_ptr<std::queue<OutgoingMessage>> _queue, Time _time) :
    filename(std::move(_filename)), queue(std::move(_queue)), time(_time)
{
}


// Recorder

Recorder::Recorder(RecorderOptions options) :
    options_(std::move(options)),
    exit_code_(0),
    queue_size_(0),
    split_count_(0),
    writing_enabled_(true)
{
}

int Recorder::Run() {
    if (options_.trigger) {
        DoTrigger();
        return 0;
    }

    if (options_.topics.empty()) {
        // Make sure limit is not specified with automatic topic subscription
        if (options_.limit > 0) {
            fprintf(stderr, "Specifing a count is not valid with automatic topic subscription.\n");
            return 1;
        }

        // Make sure topics are specified
        if (!options_.record_all && (options_.node == std::string(""))) {
            fprintf(stderr, "No topics specified.\n");
            return 1;
        }
    }

    if (!nh_.ok())
    {
        return 0;
    }

    if (options_.publish)
    {
        pub_begin_write_ = nh_.advertise<std_msgs::String>("begin_write", 1, true);
    }

    last_buffer_warn_ = Time();
    queue_ = std::make_shared<std::queue<OutgoingMessage>>();
    // Subscribe to each topic
    if (!options_.regex) {
        for (string const& topic : options_.topics) {
            shared_ptr<ros::Subscriber> sub = Subscribe(nh_, topic);
            currently_recording_.insert(topic);
            subscribers_.push_back(sub);
        }
    }

    if (!ros::Time::waitForValid(ros::WallDuration(2.0)))
    {
      ROS_WARN("/use_sim_time set to true and no clock published.  Still waiting for valid time...");
    }
    ros::Time::waitForValid();

    start_time_ = ros::Time::now();

    // Don't bother doing anything if we never got a valid time
    if (!nh_.ok())
    {
        return 0;
    }

    ros::Subscriber trigger_sub;

    // Spin up a thread for writing to the file
    boost::thread record_thread;
    if (options_.snapshot)
    {
        record_thread = boost::thread(boost::bind(&Recorder::DoRecordSnapshotter, this));

        // Subscribe to the snapshot trigger
        trigger_sub = nh_.subscribe<std_msgs::Empty>("snapshot_trigger", 100, boost::bind(&Recorder::SnapshotTrigger, this, _1));
    }
    else
    {
        record_thread = boost::thread(boost::bind(&Recorder::DoRecord, this));
    }

    ros::Timer check_master_timer;
    if (options_.record_all || options_.regex || (options_.node != std::string("")))
    {
        // check for master first
        DoCheckMaster(ros::TimerEvent(), nh_);
        check_master_timer = nh_.createTimer(ros::Duration(1.0), boost::bind(&Recorder::DoCheckMaster, this, _1, boost::ref(nh_)));
    }

    ros::AsyncSpinner s(10);
    s.start();

    record_thread.join();
    queue_condition_.notify_all();

    check_master_timer.stop();
    subscribers_.clear();
    currently_recording_.clear();
    queue_.reset();

    return exit_code_;
}

shared_ptr<ros::Subscriber> Recorder::Subscribe(ros::NodeHandle & nh, string const & topic) {
    ROS_INFO("Subscribing to %s", topic.c_str());

    shared_ptr<int> count(boost::make_shared<int>(options_.limit));
    shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());

    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
            boost::bind(&Recorder::DoQueue, this, _1, topic, sub.get(), count));
    ops.transport_hints = options_.transport_hints;
    *sub = nh.subscribe(ops);

    return sub;
}

bool Recorder::IsSubscribed(string const& topic) const {
    return currently_recording_.find(topic) != currently_recording_.end();
}

bool Recorder::ShouldSubscribeToTopic(std::string const& topic, bool from_node) {
    // ignore already known topics
    if (IsSubscribed(topic)) {
        return false;
    }

    // subtract exclusion regex, if any
    if(options_.do_exclude && boost::regex_match(topic, options_.exclude_regex)) {
        return false;
    }

    if(options_.record_all || from_node) {
        return true;
    }
    
    if (options_.regex) {
        // Treat the topics as regular expressions
  return std::any_of(
            std::begin(options_.topics), std::end(options_.topics),
            [&topic] (string const& regex_str){
                boost::regex e(regex_str);
                boost::smatch what;
                return boost::regex_match(topic, what, e, boost::match_extra);
            });
    }

    return std::find(std::begin(options_.topics), std::end(options_.topics), topic)
      != std::end(options_.topics);
}

template<class T>
std::string Recorder::TimeToStr(T ros_t)
{
    (void)ros_t;
    std::stringstream msg;
    const boost::posix_time::ptime now=
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f=
        new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

//! Callback to be invoked to save messages into a queue
void Recorder::DoQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                       string const& topic,
                       ros::Subscriber * subscriber,
                       const shared_ptr<int> & count) {
    //void Recorder::DoQueue(topic_tools::ShapeShifter::ConstPtr msg, string const& topic, shared_ptr<ros::Subscriber> subscriber, shared_ptr<int> count) {
    Time rectime = Time::now();
    
    if (options_.verbose)
    {
        cout << "Received message on topic " << subscriber->getTopic() << endl;
    }

    OutgoingMessage out(topic, msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), rectime);
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);

        queue_->push(out);
        queue_size_ += out.msg->size();
        
        // Check to see if buffer has been exceeded
        while (options_.buffer_size > 0 && queue_size_ > options_.buffer_size) {
            OutgoingMessage drop = queue_->front();
            queue_->pop();
            queue_size_ -= drop.msg->size();

            if (!options_.snapshot) {
                Time now = Time::now();
                if (now > last_buffer_warn_ + ros::Duration(5.0)) {
                    ROS_WARN("rosbag record buffer exceeded.  Dropping oldest queued message.");
                    last_buffer_warn_ = now;
                }
            }
        }
    }
  
    if (!options_.snapshot)
    {
        queue_condition_.notify_all();
    }

    // If we are book-keeping count, decrement and possibly shutdown
    if ((*count) > 0) {
        (*count)--;
        if ((*count) == 0) {
            subscriber->shutdown();
        }
    }
}

void Recorder::UpdateFilenames() {
    vector<string> parts;

    std::string prefix = options_.prefix;
    size_t ind = prefix.rfind(".bag");

    if (ind != std::string::npos && ind == prefix.size() - 4)
    {
      prefix.erase(ind);
    }

    if (prefix.length() > 0)
    {
        parts.push_back(prefix);
    }
    if (options_.append_date)
    {
        parts.push_back(TimeToStr(ros::WallTime::now()));
    }
    if (options_.split)
    {
        parts.push_back(boost::lexical_cast<string>(split_count_));
    }
    if (parts.empty())
    {
      throw rosbag::BagException("Bag filename is empty (neither of these was specified: prefix, append_date, split)");
    }

    target_filename_ = parts[0];
    for (unsigned int i = 1; i < parts.size(); i++)
    {
        target_filename_ += string("_") + parts[i];
    }

    target_filename_ += string(".bag");
    write_filename_ = target_filename_ + string(".active");
}

//! Callback to be invoked to actually do the recording
void Recorder::SnapshotTrigger(std_msgs::Empty::ConstPtr trigger) {
    (void)trigger;
    UpdateFilenames();
    
    ROS_INFO("Triggered snapshot recording with name '%s'.", target_filename_.c_str());
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, std::move(queue_), Time::now()));
        queue_ = std::make_shared<std::queue<OutgoingMessage>>();
        queue_size_ = 0;
    }

    queue_condition_.notify_all();
}

void Recorder::StartWriting() {
    bag_.setCompression(options_.compression);
    bag_.setChunkThreshold(options_.chunk_size);

    UpdateFilenames();
    try {
        bag_.open(write_filename_, rosbag::bagmode::Write);
    }
    catch (const rosbag::BagException& e) {
        ROS_ERROR("Error writing: %s", e.what());
        exit_code_ = 1;
        return;
    }
    ROS_INFO("Recording to '%s'.", target_filename_.c_str());

    if (options_.publish)
    {
        std_msgs::String msg;
        msg.data = target_filename_;
        pub_begin_write_.publish(msg);
    }
}

void Recorder::StopWriting() {
    ROS_INFO("Closing '%s'.", target_filename_.c_str());
    bag_.close();
    (void) rename(write_filename_.c_str(), target_filename_.c_str());
}

void Recorder::CheckNumSplits()
{
    if(options_.max_splits>0)
    {
        current_files_.push_back(target_filename_);
        if(current_files_.size()>options_.max_splits)
        {
            int err = unlink(current_files_.front().c_str());
            if(err != 0)
            {
                ROS_ERROR("Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
            }
            current_files_.pop_front();
        }
    }
}

bool Recorder::CheckSize()
{
    if (options_.max_size > 0)
    {
        if (bag_.getSize() > options_.max_size)
        {
            if (options_.split)
            {
                StopWriting();
                split_count_++;
                CheckNumSplits();
                StartWriting();
            } else {
                return true;
            }
        }
    }
    return false;
}

bool Recorder::CheckDuration(const ros::Time& t)
{
    if (options_.max_duration > ros::Duration(0))
    {
        if (t - start_time_ > options_.max_duration)
        {
            if (options_.split)
            {
                while (start_time_ + options_.max_duration < t)
                {
                    StopWriting();
                    split_count_++;
                    CheckNumSplits();
                    start_time_ += options_.max_duration;
                    StartWriting();
                }
            } else {
                return true;
            }
        }
    }
    return false;
}


//! Thread that actually does writing to file.
void Recorder::DoRecord() {
    // Open bag file for writing
    StartWriting();

    // Schedule the disk space check
    warn_next_ = ros::WallTime();

    try
    {
        CheckDisk();
    }
    catch (const rosbag::BagException& ex)
    {
        ROS_ERROR_STREAM(ex.what());
        exit_code_ = 1;
        StopWriting();
        return;
    }

    check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
    ros::NodeHandle nh;
    while (nh.ok() || !queue_->empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!nh.ok()) {
                lock.release()->unlock();
                finished = true;
                break;
            }
            boost::xtime xt {};
#if BOOST_VERSION >= 105000
            boost::xtime_get(&xt, boost::TIME_UTC_);
#else
            boost::xtime_get(&xt, boost::TIME_UTC);
#endif
            xt.nsec += 250000000;
            queue_condition_.timed_wait(lock, xt);
            if (CheckDuration(ros::Time::now()))
            {
                finished = true;
                break;
            }
        }
        if (finished)
        {
            break;
        }

        OutgoingMessage out = queue_->front();
        queue_->pop();
        queue_size_ -= out.msg->size();
        
        lock.release()->unlock();
        
        if (CheckSize())
        {
            break;
        }

        if (CheckDuration(out.time))
        {
            break;
        }

        try
        {
            if (ScheduledCheckDisk() && CheckLogging())
            {
                bag_.write(out.topic, out.time, *out.msg, out.connection_header);
            }
        }
        catch (const rosbag::BagException& ex)
        {
            ROS_ERROR_STREAM(ex.what());
            exit_code_ = 1;
            break;
        }
    }

    StopWriting();
}

void Recorder::DoRecordSnapshotter() {
    ros::NodeHandle nh;
  
    while (nh.ok() || !queue_queue_.empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);
        while (queue_queue_.empty()) {
            if (!nh.ok())
            {
                return;
            }
            queue_condition_.wait(lock);
        }
        
        OutgoingQueue out_queue = queue_queue_.front();
        queue_queue_.pop();
        
        lock.release()->unlock();
        
        string target_filename = out_queue.filename;
        string write_filename  = target_filename + string(".active");
        
        try {
            bag_.open(write_filename, rosbag::bagmode::Write);
        }
        catch (const rosbag::BagException& ex) {
            ROS_ERROR("Error writing: %s", ex.what());
            return;
        }

        while (!out_queue.queue->empty()) {
            OutgoingMessage out = out_queue.queue->front();
            out_queue.queue->pop();

            bag_.write(out.topic, out.time, *out.msg);
        }

        StopWriting();
    }
}

void Recorder::DoCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle) {
    (void) e;
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics)) {
        for (ros::master::TopicInfo const& t : topics) {
            if (ShouldSubscribeToTopic(t.name)) {
                shared_ptr<ros::Subscriber> sub = Subscribe(node_handle, t.name);
                currently_recording_.insert(t.name);
                subscribers_.push_back(sub);
            }
        }
    }
    
    if (options_.node != std::string(""))
    {

      XmlRpc::XmlRpcValue req;
      req[0] = ros::this_node::getName();
      req[1] = options_.node;
      XmlRpc::XmlRpcValue resp;
      XmlRpc::XmlRpcValue payload;

      if (ros::master::execute("lookupNode", req, resp, payload, true))
      {
        std::string peer_host;
        uint32_t peer_port;

        if (!ros::network::splitURI(static_cast<std::string>(resp[2]), peer_host, peer_port))
        {
          ROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
        } else {

          XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
          XmlRpc::XmlRpcValue req2;
          XmlRpc::XmlRpcValue resp2;
          req2[0] = ros::this_node::getName();
          c.execute("getSubscriptions", req2, resp2);
          
          if (!c.isFault() && resp2.valid() && resp2.size() > 0 && static_cast<int>(resp2[0]) == 1)
          {
            for(int i = 0; i < resp2[2].size(); i++)
            {
              if (ShouldSubscribeToTopic(resp2[2][i][0], true))
              {
                shared_ptr<ros::Subscriber> sub = Subscribe(node_handle, resp2[2][i][0]);
                currently_recording_.insert(resp2[2][i][0]);
                subscribers_.push_back(sub);
              }
            }
          } else {
            ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[2]).c_str());
          }
        }
      }
    }
}

void Recorder::DoTrigger() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
    pub.publish(std_msgs::Empty());

    ros::Timer terminate_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&ros::shutdown));
    ros::spin();
}

bool Recorder::ScheduledCheckDisk() {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (ros::WallTime::now() < check_disk_next_)
    {
        return true;
    }

    check_disk_next_ += ros::WallDuration().fromSec(20.0);
    return CheckDisk();
}

bool Recorder::CheckDisk() {
#if BOOST_FILESYSTEM_VERSION < 3
    struct statvfs fiData;
    if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0)
    {
        ROS_WARN("Failed to check filesystem stats.");
        return true;
    }
    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < options_.min_space)
    {
        ROS_ERROR("Less than %s of space free on disk with '%s'.  Disabling recording.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
        writing_enabled_ = false;
        return false;
    }
    else if (free_space < 5 * options_.min_space)
    {
        ROS_WARN("Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
    }
    else
    {
        writing_enabled_ = true;
    }
#else
    boost::filesystem::path p(boost::filesystem::system_complete(bag_.getFileName().c_str()));
    p = p.parent_path();
    boost::filesystem::space_info info {};
    try
    {
        info = boost::filesystem::space(p);
    }
    catch (const boost::filesystem::filesystem_error& e) 
    { 
        ROS_WARN("Failed to check filesystem stats [%s].", e.what());
        writing_enabled_ = false;
        return false;
    }
    if ( info.available < options_.min_space)
    {
        writing_enabled_ = false;
        throw rosbag::BagException("Less than " + options_.min_space_str + " of space free on disk with " + bag_.getFileName() + ". Disabling recording.");
    }
    else if (info.available < 5 * options_.min_space)
    {
        ROS_WARN("Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
        writing_enabled_ = true;
    }
    else
    {
        writing_enabled_ = true;
    }
#endif
    return true;
}

bool Recorder::CheckLogging() {
    if (writing_enabled_)
    {
        return true;
    }

    ros::WallTime now = ros::WallTime::now();
    if (now >= warn_next_) {
        warn_next_ = now + ros::WallDuration().fromSec(5.0);
        ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
