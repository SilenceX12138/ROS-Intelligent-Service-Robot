/*********************************************************************
 *
 *  Copyright (c) 2014, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#include <sstream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <tf2_web_republisher/TFSubscriptionAction.h>

#include <tf2_web_republisher/RepublishTFs.h>
#include <tf2_web_republisher/TFArray.h>

#include "tf_pair.h"

class TFRepublisher
{
protected:
  typedef actionlib::ActionServer<tf2_web_republisher::TFSubscriptionAction> TFTransformServer;
  typedef TFTransformServer::GoalHandle GoalHandle;

  typedef tf2_web_republisher::RepublishTFs::Request Request;
  typedef tf2_web_republisher::RepublishTFs::Response Response;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  TFTransformServer as_;
  ros::ServiceServer tf_republish_service_;

  // base struct that holds information about the TFs
  // a client (either Service or Action) has subscribed to
  struct ClientInfo
  {
    std::vector<TFPair> tf_subscriptions_;
    unsigned int client_ID_;
    ros::Timer  timer_;
  };

  // struct for Action client info
  struct ClientGoalInfo : ClientInfo
  {
    GoalHandle handle;
  };

  // struct for Service client info
  struct ClientRequestInfo : ClientInfo
  {
    ros::Publisher pub_;
    ros::Duration unsub_timeout_;
    ros::Timer unsub_timer_;
  };

  std::list<boost::shared_ptr<ClientGoalInfo> > active_goals_;
  boost::mutex goals_mutex_;

  std::list<boost::shared_ptr<ClientRequestInfo> > active_requests_;
  boost::mutex requests_mutex_;

  // tf2 buffer and transformer
#if ROS_VERSION_MINOR < 10
  tf2::Buffer tf_buffer_;
  tf2::TransformListener tf_listener_;
#else
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
#endif
  boost::mutex tf_buffer_mutex_;

  unsigned int client_ID_count_;

public:

  TFRepublisher(const std::string& name) :
    nh_(),
    as_(nh_,
        name,
        boost::bind(&TFRepublisher::goalCB, this, _1),
        boost::bind(&TFRepublisher::cancelCB, this, _1),
        false),
    priv_nh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    client_ID_count_(0)
  {
    tf_republish_service_ = nh_.advertiseService("republish_tfs",
                                                 &TFRepublisher::requestCB,
                                                 this);
    as_.start();
  }

  ~TFRepublisher() {}



  void cancelCB(GoalHandle gh)
  {
    boost::mutex::scoped_lock l(goals_mutex_);

    ROS_DEBUG("GoalHandle canceled");

    // search for goal handle and remove it from active_goals_ list
    for(std::list<boost::shared_ptr<ClientGoalInfo> >::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      ClientGoalInfo& info = **it;
      if(info.handle == gh)
      {
        it = active_goals_.erase(it);
        info.timer_.stop();
        info.handle.setCanceled();
        return;
      }
      else
        ++it;
    }
  }

  const std::string cleanTfFrame( const std::string frame_id ) const
  {
    if ( frame_id[0] == '/' )
    {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  /**
   * Set up the contents of \p tf_subscriptions_ in
   * a ClientInfo struct
   */
  void setSubscriptions(boost::shared_ptr<ClientInfo> info,
                        const std::vector<std::string>& source_frames,
                        const std::string& target_frame_,
                        float angular_thres,
                        float trans_thres) const
  {
    std::size_t request_size_ = source_frames.size();
    info->tf_subscriptions_.resize(request_size_);

    for (std::size_t i=0; i<request_size_; ++i )
    {
      TFPair& tf_pair = info->tf_subscriptions_[i];

      std::string source_frame = cleanTfFrame(source_frames[i]);
      std::string target_frame = cleanTfFrame(target_frame_);

      tf_pair.setSourceFrame(source_frame);
      tf_pair.setTargetFrame(target_frame);
      tf_pair.setAngularThres(angular_thres);
      tf_pair.setTransThres(trans_thres);
    }
  }

  void goalCB(GoalHandle gh)
  {
    ROS_DEBUG("GoalHandle request received");

    // accept new goals
    gh.setAccepted();

    // get goal from handle
    const tf2_web_republisher::TFSubscriptionGoal::ConstPtr& goal = gh.getGoal();

    // generate goal_info struct
    boost::shared_ptr<ClientGoalInfo> goal_info = boost::make_shared<ClientGoalInfo>();
    goal_info->handle = gh;
    goal_info->client_ID_ = client_ID_count_++;

    // add the tf_subscriptions to the ClientGoalInfo object
    setSubscriptions(goal_info,
                     goal->source_frames,
                     goal->target_frame,
                     goal->angular_thres,
                     goal->trans_thres);

    goal_info->timer_ = nh_.createTimer(ros::Duration(1.0 / goal->rate),
                                        boost::bind(&TFRepublisher::processGoal, this, goal_info, _1));

    {
      boost::mutex::scoped_lock l(goals_mutex_);
      // add new goal to list of active goals/clients
      active_goals_.push_back(goal_info);
    }

  }

  bool requestCB(Request& req, Response& res)
  {
    ROS_DEBUG("RepublishTF service request received");
    // generate request_info struct
    boost::shared_ptr<ClientRequestInfo> request_info = boost::make_shared<ClientRequestInfo>();

    request_info->client_ID_ = client_ID_count_;
    std::stringstream topicname;
    topicname << "tf_repub_" << client_ID_count_++;

    request_info->pub_ = priv_nh_.advertise<tf2_web_republisher::TFArray>(topicname.str(), 10, true);

    // add the tf_subscriptions to the ClientGoalInfo object
    setSubscriptions(request_info,
                     req.source_frames,
                     req.target_frame,
                     req.angular_thres,
                     req.trans_thres);

    request_info->unsub_timeout_ = req.timeout;
    request_info->unsub_timer_ = nh_.createTimer(request_info->unsub_timeout_,
                                                 boost::bind(&TFRepublisher::unadvertiseCB, this, request_info, _1),
                                                 true); // only fire once

    request_info->timer_ = nh_.createTimer(ros::Duration(1.0 / req.rate),
                                           boost::bind(&TFRepublisher::processRequest, this, request_info, _1));

    {
      boost::mutex::scoped_lock l(requests_mutex_);
      // add new request to list of active requests
      active_requests_.push_back(request_info);
    }
    res.topic_name = request_info->pub_.getTopic();
    ROS_INFO_STREAM("Publishing requested TFs on topic " << res.topic_name);

    return true;
  }

  void unadvertiseCB(boost::shared_ptr<ClientRequestInfo> request_info, const ros::TimerEvent&)
  {
    ROS_INFO_STREAM("No subscribers on tf topic for request "
                     << request_info->client_ID_
                     << " for " << request_info->unsub_timeout_.toSec()
                     << " seconds. Unadvertising topic:"
                     << request_info->pub_.getTopic());
    request_info->pub_.shutdown();
    request_info->unsub_timer_.stop();
    request_info->timer_.stop();

    // search for ClientRequestInfo struct and remove it from active_requests_ list
    for(std::list<boost::shared_ptr<ClientRequestInfo> >::iterator it = active_requests_.begin(); it != active_requests_.end(); ++it)
    {
      ClientRequestInfo& info = **it;
      if(info.pub_ == request_info->pub_)
      {
        active_requests_.erase(it);
        return;
      }
    }
  }

  void updateSubscriptions(std::vector<TFPair>& tf_subscriptions,
                           std::vector<geometry_msgs::TransformStamped>& transforms)
  {
    // iterate over tf_subscription vector
    std::vector<TFPair>::iterator it ;
    std::vector<TFPair>::const_iterator end = tf_subscriptions.end();

    for (it=tf_subscriptions.begin(); it!=end; ++it)
    {
      geometry_msgs::TransformStamped transform;

      try
      {
        // protecting tf_buffer
        boost::mutex::scoped_lock lock (tf_buffer_mutex_);

        // lookup transformation for tf_pair
        transform = tf_buffer_.lookupTransform(it->getTargetFrame(),
                                               it->getSourceFrame(),
                                               ros::Time(0));

        // If the transform broke earlier, but worked now (we didn't get
        // booted into the catch block), tell the user all is well again
        if (!it->is_okay)
        {
          it->is_okay = true;
          ROS_INFO_STREAM("Transform from "
                          << it->getSourceFrame()
                          << " to "
                          << it->getTargetFrame()
                          << " is working again at time "
                          << transform.header.stamp.toSec());
        }
        // update tf_pair with transformtion
        it->updateTransform(transform);
      }
      catch (tf2::TransformException ex)
      {
        // Only log an error if the transform was okay before
        if (it->is_okay)
        {
          it->is_okay = false;
          ROS_ERROR("%s", ex.what());
        }
      }

      // check angular and translational thresholds
      if (it->updateNeeded())
      {
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = it->getTargetFrame();
        transform.child_frame_id = it->getSourceFrame();

        // notify tf_subscription that a network transmission has been triggered
        it->transmissionTriggered();

        // add transform to the array
        transforms.push_back(transform);
      }
    }
  }

  void processGoal(boost::shared_ptr<ClientGoalInfo> goal_info, const ros::TimerEvent& )
  {
    tf2_web_republisher::TFSubscriptionFeedback feedback;

    updateSubscriptions(goal_info->tf_subscriptions_,
                        feedback.transforms);

    if (feedback.transforms.size() > 0)
    {
      // publish feedback
      goal_info->handle.publishFeedback(feedback);
      ROS_DEBUG("Client %d: TF feedback published:", goal_info->client_ID_);
    } else
    {
      ROS_DEBUG("Client %d: No TF frame update needed:", goal_info->client_ID_);
    }
  }

  void processRequest(boost::shared_ptr<ClientRequestInfo> request_info, const ros::TimerEvent& )
  {
    if (request_info->pub_.getNumSubscribers() == 0)
    {
      request_info->unsub_timer_.start();
    }
    else
    {
      request_info->unsub_timer_.stop();
    }

    tf2_web_republisher::TFArray array_msg;
    updateSubscriptions(request_info->tf_subscriptions_,
                        array_msg.transforms);

    if (array_msg.transforms.size() > 0)
    {
      // publish TFs
      request_info->pub_.publish(array_msg);
      ROS_DEBUG("Request %d: TFs published:", request_info->client_ID_);
    }
    else
    {
      ROS_DEBUG("Request %d: No TF frame update needed:", request_info->client_ID_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf2_web_republisher");

  TFRepublisher tf2_web_republisher(ros::this_node::getName());
  ros::spin();

  return 0;
}
