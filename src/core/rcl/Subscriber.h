/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_SUBSCRIBER_H_
#define ROS_SUBSCRIBER_H_

#include "Node.h"
#include <string.h>
#include <stdio.h>
#include <TopicReader.h>
#include <XMLRPCServer.h>

// TODO: Why does STM32 crash if max_subscribers>=20? Not enough memory?
#define MAX_SUBSCRIBERS 10
namespace ros
{

/* Base class for objects subscribers. */
class Subscriber_
{
public:
  //virtual void callback()=0;

  virtual void deserialize(unsigned char *data) = 0;
  // id_ is set by NodeHandle when we advertise
  int id_;

  virtual const char * getMsgType() = 0;
  virtual const char * getMsgMD5() = 0;
  const char * topic;
  static Subscriber_** list;
  static int lastSubscriberIndex;
};


/* Actual subscriber, templated on message type. */
template<typename MsgT>
class Subscriber: public Subscriber_
{
public:
  typedef void(*CallbackT)(const MsgT&);
  MsgT msg;

  Subscriber(Node* node, const char * topic_name, CallbackT cb) :
    cb_(cb)
  {
    char taskName[32];
    topic = topic_name;
    list[++lastSubscriberIndex] = this;

    TopicReader* tr = XMLRPCServer::registerSubscriber(node->name, topic, msg.getMD5(), msg.getType());
    this->callback = cb;
    tr->addCallback(subCallback, this);
  };

  static void subCallback(void* data, void* obj)
  {
    Subscriber* self = (Subscriber*) obj;
    MsgT msg;
    msg.deserialize((unsigned char*)data);
    self->callback(msg);
  }

  virtual void deserialize(unsigned char* data)
  {
    msg.deserialize(data);
  }
  void(*callback)(const MsgT& msg);
  virtual const char * getMsgType() { return this->msg.getType(); }
  virtual const char * getMsgMD5() { return this->msg.getMD5(); }

private:
  CallbackT cb_;
};

}

#endif
