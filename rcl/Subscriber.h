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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

// TODO: Why does STM32 crash if max_subscribers>=20? Not enough memory?
#define MAX_SUBSCRIBERS 10
namespace ros {

  /* Base class for objects subscribers. */
  class Subscriber_
  {
    public:
      virtual void callback()=0;
      virtual void deserialize(unsigned char *data)=0;

      // id_ is set by NodeHandle when we advertise 
      int id_;

      virtual const char * getMsgType()=0;
      virtual const char * getMsgMD5()=0;
      const char * topic;
      xSemaphoreHandle signal, dataAccess;
      static Subscriber_** list;
      static int lastSubscriberIndex;
  };


  /* Actual subscriber, templated on message type. */
  template<typename MsgT>
  class Subscriber: public Subscriber_{
    public:
      typedef void(*CallbackT)(const MsgT&);
      MsgT msg;

      Subscriber(Node* node, const char * topic_name, CallbackT cb) :
        cb_(cb)
      {
    	  char taskName[32];
    	  topic = topic_name;
    	  list[++lastSubscriberIndex] = this;
		vSemaphoreCreateBinary(signal);
		vSemaphoreCreateBinary(dataAccess);
		xSemaphoreTake(signal, 0); // do this operation to initialize the semaphore with 0 resources
		sprintf(taskName, "subscriber_%s_%s", node->name, topic);
		os_printf("name:%s\n", taskName);
		xTaskCreate(subscriberCallbackTask, (const signed char*)taskName, 128, (void*) this, tskIDLE_PRIORITY + 2, NULL);

      };

      virtual void deserialize(unsigned char* data)
      {
    	  msg.deserialize(data);
      }

      virtual void callback(){
        this->cb_(msg);
      }

      virtual const char * getMsgType(){ return this->msg.getType(); }
      virtual const char * getMsgMD5(){ return this->msg.getMD5(); }

      static void subscriberCallbackTask(void* params)
      {
    	Subscriber_* subscriber = (Subscriber_*) params;

      	if (subscriber != NULL)
      	{
      		while(1)
      		{
      			if (xSemaphoreTake(subscriber->signal, portMAX_DELAY))
      			{
      				subscriber->callback();
      				xSemaphoreGive(subscriber->dataAccess); // the RX task is allowed to proceed accessing shared data now.
      			}
      		}
      	}
      	else
      	{
      		os_printf("Subscriber is null! Deleting callback task.");
      		vTaskDelete(NULL); // kill yourself
      	}
      }

    private:
      CallbackT cb_;
  };

}

#endif
