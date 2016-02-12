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

#include "evologics_driver/AcousticModemPayload.h"


namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;

class Session
{
public:
  Session()
    : client_version(PROTOCOL_UNKNOWN),
      client_version_try(PROTOCOL_VER2)
  {

    callbacks_[rosserial_msgs::TopicInfo::ID_SUBSCRIBER]
        = boost::bind(&Session::setup_subscriber, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT+rosserial_msgs::TopicInfo::ID_PUBLISHER]
        = boost::bind(&Session::setup_service_client_publisher, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT+rosserial_msgs::TopicInfo::ID_SUBSCRIBER]
        = boost::bind(&Session::setup_service_client_subscriber, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_LOG]
        = boost::bind(&Session::handle_log, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_TIME]
        = boost::bind(&Session::handle_time, this, _1);
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
    //generic_sub_ = nh_.subscribe<evologics_driver::AcousticModemPayload>("im/in", 1, &Session::genericCb, this);
    generic_pub_ = nh_.advertise<evologics_driver::AcousticModemPayload>("im/out", 1);
  }

  enum Version {
    PROTOCOL_UNKNOWN = 0,
    PROTOCOL_VER1 = 1,
    PROTOCOL_VER2 = 2,
    PROTOCOL_MAX
  };

private:
  //// RECEIVING MESSAGES ////

  void genericCb(const evologics_driver::AcousticModemPayload& msg) {

    // TODO: convert the message to serial.
    //std::string payload = msg.payload;

    //
    std::vector<uint8_t> mem;
    size_t bytes_transferred = 10;
    ros::serialization::IStream stream(&mem[0], bytes_transferred);

    uint16_t topic_id, length;
    uint8_t length_checksum;
    if (client_version == PROTOCOL_VER2) {
      // Complex header with checksum byte for length field.
      stream >> length >> length_checksum;
      if (length_checksum + checksum(length) != 0xff) {
        uint8_t csl = checksum(length);
        ROS_WARN("Bad message header length checksum. Dropping message from client. T%d L%d C%d %d", topic_id, length, length_checksum, csl);
        return;
      } else {
        stream >> topic_id;
      }
    } else if (client_version == PROTOCOL_VER1) {
      // Simple header in VER1 protocol.
      stream >> topic_id >> length;
    }
    ROS_DEBUG("Received message header with length %d and topic_id=%d", length, topic_id);

    // Read message length + checksum byte.
    read_body(stream, topic_id);
  }

  void read_body(ros::serialization::IStream& stream, uint16_t topic_id) {
    ROS_DEBUG("Received body of length %d for message on topic %d.", stream.getLength(), topic_id);

    ros::serialization::IStream checksum_stream(stream.getData(), stream.getLength());

    uint8_t msg_checksum = checksum(checksum_stream) + checksum(topic_id);
    if (client_version == PROTOCOL_VER1) {
      msg_checksum += checksum(stream.getLength() - 1);
    }

    if (msg_checksum != 0xff) {
      ROS_WARN("Rejecting message on topicId=%d, length=%d with bad checksum.", topic_id, stream.getLength());
    } else {
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
        // TODO: Resynchronize on multiples?
      }
    }
  }


  //// SENDING MESSAGES ////

  // This function build the message
  void write_message(Buffer& message,
                     const uint16_t topic_id,
                     Session::Version version) {
    uint8_t overhead_bytes = 0;
    switch(version) {
      case PROTOCOL_VER2: overhead_bytes = 8; break;
      case PROTOCOL_VER1: overhead_bytes = 7; break;
      default:
        ROS_WARN("Aborting write_message: protocol unspecified.");
    }

    uint16_t length = overhead_bytes + message.size();
    BufferPtr buffer_ptr(new Buffer(length));

    uint8_t msg_checksum;
    ros::serialization::IStream checksum_stream(message.size() > 0 ? &message[0] : NULL, message.size());

    ros::serialization::OStream stream(&buffer_ptr->at(0), buffer_ptr->size());
    if (version == PROTOCOL_VER2) {
      uint8_t msg_len_checksum = 255 - checksum(message.size());
      stream << (uint16_t)0xfeff << (uint16_t)message.size() << msg_len_checksum << topic_id;
      msg_checksum = 255 - (checksum(checksum_stream) + checksum(topic_id));
    } else if (version == PROTOCOL_VER1) {
      stream << (uint16_t)0xffff << topic_id << (uint16_t)message.size();
      msg_checksum = 255 - (checksum(checksum_stream) + checksum(topic_id) + checksum(message.size()));
    }
    memcpy(stream.advance(message.size()), &message[0], message.size());
    stream << msg_checksum;

    // TODO: convert istream to payload message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();
    generic_pub_.publish(msg);
  }

  //// SYNC WATCHDOG ////

  void required_topics_check() {
    if (ros::param::has("~require")) {
      if (!check_set("~require/publishers", publishers_, true)) {
        ROS_WARN("Connected client failed to establish the publishers and subscribers dictated by require parameter.");
      }
      /*
      if (!check_set("~require/publishers", publishers_, true) ||
          !check_set("~require/subscribers", subscribers_, false)) {
        ROS_WARN("Connected client failed to establish the publishers and subscribers dictated by require parameter.");
      }
      */
    }
  }

  template<typename M>
  bool check_set(std::string param_name, M map, bool is_pub) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct );
    //for (int i = 0; i < param_list.size(); ++i) {
    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++) {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      rosserial_msgs::TopicInfo topic_info;

      topic_info.topic_id = (int)data["topic_id"];
      topic_info.topic_name = (std::string)data["topic_name"];
      topic_info.message_type = (std::string)data["message_type"];
      topic_info.buffer_size = (int)data["buffer_size"];
      //topic_info.md5sum = (std::string)data["md5sum"];

      ROS_INFO("=======TOPIC_INFO=======");
      ROS_INFO_STREAM("Size: "          << data.size());
      ROS_INFO_STREAM("Topic_id: "      << topic_info.topic_id);
      ROS_INFO_STREAM("Topic_name: "    << topic_info.topic_name);
      ROS_INFO_STREAM("Message_type: "  << topic_info.message_type);
      ROS_INFO_STREAM("Buffer_size: "   << topic_info.buffer_size);
      //ROS_INFO_STREAM("md5sum: "        << topic_info.md5sum);
      ROS_INFO("========================");

      if (is_pub)
        setup_publisher(topic_info);

    }
    return true;
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

  void setup_subscriber(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    SubscriberPtr sub(new Subscriber(nh_, topic_info,
        boost::bind(&Session::write_message, this, _1, topic_info.topic_id, client_version)));
    subscribers_[topic_info.topic_id] = sub;
  }

  // When the rosserial client creates a ServiceClient object (and/or when it registers that object with the NodeHandle)
  // it creates a publisher (to publish the service request message to us) and a subscriber (to receive the response)
  // the service client callback is attached to the *subscriber*, so when we receive the service response
  // and wish to send it over the socket to the client,
  // we must attach the topicId that came from the service client subscriber message

  void setup_service_client_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    if (!services_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, _2, client_version)));
      services_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
    if (services_[topic_info.topic_name]->getRequestMessageMD5() != topic_info.md5sum) {
      ROS_WARN("Service client setup: Request message MD5 mismatch between rosserial client and ROS");
    } else {
      ROS_DEBUG("Service client %s: request message MD5 successfully validated as %s",
        topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
  }

  void setup_service_client_subscriber(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    if (!services_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, _2, client_version)));
      services_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
    // see above comment regarding the service client callback for why we set topic_id here
    services_[topic_info.topic_name]->setTopicId(topic_info.topic_id);
    if (services_[topic_info.topic_name]->getResponseMessageMD5() != topic_info.md5sum) {
      ROS_WARN("Service client setup: Response message MD5 mismatch between rosserial client and ROS");
    } else {
      ROS_DEBUG("Service client %s: response message MD5 successfully validated as %s",
        topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
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

  void handle_time(ros::serialization::IStream& stream) {
    std_msgs::Time time;
    time.data = ros::Time::now();

    size_t length = ros::serialization::serializationLength(time);
    std::vector<uint8_t> message(length);

    ros::serialization::OStream ostream(&message[0], length);
    ros::serialization::Serializer<std_msgs::Time>::write(ostream, time);

    write_message(message, rosserial_msgs::TopicInfo::ID_TIME, client_version);
  }

  ros::NodeHandle nh_;
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
