/**
 *
 *  \file
 *  \brief      Class representing a session between this node and a
 *              templated rosserial client.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_SESSION_H
#define ROSSERIAL_SERVER_SESSION_H

#include <map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <rosserial_msgs/Log.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>

#include "rosserial_server/async_read_buffer.h"
#include "rosserial_server/topic_handlers.h"

#include "evologics_ros/AcousticModemPayload.h"


namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;

class Session
{
public:
  Session()
    : client_version(PROTOCOL_UNKNOWN),
      client_version_try(PROTOCOL_VER2),
      nhp_("~")
  {

    callbacks_[rosserial_msgs::TopicInfo::ID_LOG]
        = boost::bind(&Session::handle_log, this, _1);
  }

  virtual ~Session()
  {
    ROS_INFO("Ending session.");
  }

  void start()
  {
    ROS_INFO("Starting session.");

    // List the required topics
    required_topics_check();

    // Subscribe/publish to ros topic
    generic_sub_ = nhp_.subscribe<evologics_ros::AcousticModemPayload>("input", 1, &Session::genericCb, this);
    generic_pub_ = nhp_.advertise<evologics_ros::AcousticModemPayload>("output", 1);
  }

  enum Version {
    PROTOCOL_UNKNOWN = 0,
    PROTOCOL_VER1 = 1,
    PROTOCOL_VER2 = 2,
    PROTOCOL_MAX
  };

private:
  //// RECEIVING MESSAGES ////

  void genericCb(const evologics_ros::AcousticModemPayload::ConstPtr& msg) {

    // Extract message
    std::string payload = msg->payload;
    std::vector<uint8_t> mem(payload.begin(), payload.end());

    // Extract topic id
    uint16_t topic_id = (uint16_t)mem[0];

    // Serialize
    ros::serialization::IStream stream(&mem[1], mem.size()-1);

    if (callbacks_.count(topic_id) == 1) {
      try {
        callbacks_[topic_id](stream);
      } catch(ros::serialization::StreamOverrunException e) {
        if (topic_id < 100) {
          ROS_ERROR("Buffer overrun when attempting to parse setup message.");
          ROS_ERROR_ONCE("Is this firmware from a pre-Groovy rosserial?");
        } else {
          ROS_WARN("Buffer overrun when attempting to parse user message.");
        }
      }
    } else {
      ROS_WARN("Received message with unrecognized topicId (%d).", topic_id);
    }
  }


  //// SENDING MESSAGES ////

  // This function build the message
  void write_message(Buffer& message,
                     rosserial_msgs::TopicInfo topic_info,
                     Session::Version version) {

    // Insert topic ID
    uint8_t topic_id = (uint8_t)topic_info.topic_id;
    message.insert(message.begin(), topic_id);

    // Convert stream to payload message
    evologics_ros::AcousticModemPayload msg;
    msg.address = (uint8_t)topic_info.address;

    std::string payload(message.begin(), message.end());
    msg.payload = payload;
    generic_pub_.publish(msg);
  }

  //// SYNC WATCHDOG ////

  void required_topics_check() {
    // Get node address
    XmlRpc::XmlRpcValue param;
    ros::param::get("~origin_address", param);
    int origin = (int)param;
    ROS_INFO_STREAM("Origin: " << origin);

    // Publishers
    if (ros::param::has("~topics")) {
      rosserial_msgs::TopicInfo topic_info;
      topic_info = fill_topic_info("~topics");
      if (topic_info.address != origin) setup_publisher(topic_info);
      else setup_subscriber(topic_info);
    }
    else ROS_WARN("Failed to establish the TOPIC connections dictated by require parameter.");


    // Services
    if (ros::param::has("~services")) {
      rosserial_msgs::TopicInfo topic_info;
      topic_info = fill_service_info("~services");
      if (topic_info.address != origin) setup_service_client_publisher(topic_info);
      else setup_service_client_subscriber(topic_info);
    }
    else ROS_WARN("Failed to establish the SERVICE connections dictated by require parameter.");
  }


  rosserial_msgs::TopicInfo fill_topic_info(std::string param_name) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++) {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      rosserial_msgs::TopicInfo topic_info;
      topic_info.topic_id = (int)data["topic_id"];
      topic_info.topic_name = (std::string)data["topic_name"];
      topic_info.message_type = (std::string)data["message_type"];
      topic_info.address = (int)data["destination_address"];

      return topic_info;
    }
  }

  rosserial_msgs::TopicInfo fill_service_info(std::string param_name) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++) {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      rosserial_msgs::TopicInfo topic_info;
      topic_info.topic_id = (int)data["service_id"];
      topic_info.topic_name = (std::string)data["service_name"];
      topic_info.address = (int)data["owner_address"];

      return topic_info;
    }
  }



  static uint8_t checksum(ros::serialization::IStream& stream) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < stream.getLength(); ++i) {
      sum += stream.getData()[i];
    }
    return sum;
  }

  static uint8_t checksum(uint16_t val) {
    return (val >> 8) + val;
  }

  //// RECEIVED MESSAGE HANDLERS ////

  void setup_publisher(rosserial_msgs::TopicInfo topic_info) {
    PublisherPtr pub(new Publisher(nh_, topic_info));
    publishers_[topic_info.topic_id] = pub;
    callbacks_[topic_info.topic_id] = boost::bind(&Publisher::handle, pub, _1);
  }

  void setup_subscriber(rosserial_msgs::TopicInfo topic_info) {
    SubscriberPtr sub(new Subscriber(nh_, topic_info,
        boost::bind(&Session::write_message, this, _1, topic_info, client_version)));
    subscribers_[topic_info.topic_id] = sub;
  }

  // When the rosserial client creates a ServiceClient object (and/or when it registers that object with the NodeHandle)
  // it creates a publisher (to publish the service request message to us) and a subscriber (to receive the response)
  // the service client callback is attached to the *subscriber*, so when we receive the service response
  // and wish to send it over the socket to the client,
  // we must attach the topicId that came from the service client subscriber message

  void setup_service_client_publisher(rosserial_msgs::TopicInfo topic_info) {
    if (!services_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, topic_info, client_version)));
      services_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
  }

  void setup_service_client_subscriber(rosserial_msgs::TopicInfo topic_info) {
    if (!services_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, topic_info, client_version)));
      services_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
    // see above comment regarding the service client callback for why we set topic_id here
    services_[topic_info.topic_name]->setTopicId(topic_info.topic_id);
  }

  void handle_log(ros::serialization::IStream& stream) {
    rosserial_msgs::Log l;
    ros::serialization::Serializer<rosserial_msgs::Log>::read(stream, l);
    if(l.level == rosserial_msgs::Log::ROSDEBUG) ROS_DEBUG("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::INFO) ROS_INFO("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::WARN) ROS_WARN("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::ERROR) ROS_ERROR("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::FATAL) ROS_FATAL("%s", l.msg.c_str());
  }


  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber generic_sub_;
  ros::Publisher generic_pub_;

  Session::Version client_version;
  Session::Version client_version_try;

  std::map< uint16_t, boost::function<void(ros::serialization::IStream)> > callbacks_;
  std::map< uint16_t, PublisherPtr > publishers_;
  std::map< uint16_t, SubscriberPtr > subscribers_;
  std::map<std::string, ServiceClientPtr> services_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_SESSION_H
