#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "queue.h" // Might not be a robust solution!
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <map>
#include <string>



#include "Message.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#define RX_PORT 32001
#define TX_PORT 32002
#define TX_QUEUE_MSG_SIZE 128
#define TX_QUEUE_LEN 2000
#define TX_IP_ADDR "10.3.84.99"

char txQueueBuffer[TX_QUEUE_MSG_SIZE * TX_QUEUE_LEN];
queue_t txQueue = QUEUE_INITIALIZER(txQueueBuffer);

std::map<std::string, std::pair<ros::Publisher, MessageType> > publishers;

Message m;

void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
   char topicName[] = "sub";
   int topicLen = strlen(topicName);
   
   

   ROS_INFO("I heard: [%d]", msg->data);
   unsigned char buffer[TX_QUEUE_MSG_SIZE];
   buffer[0] = (unsigned char) topicLen;
   memcpy(&buffer[1], topicName, topicLen);
   unsigned int offset = 1 + topicLen;
   *((int*)m.data) = msg->data;
   offset += serialize_int32(&m, &buffer[offset]);
   queue_enqueue(&txQueue, (char*) buffer);

}

void* RXThread(void* arg)
{
	int argc = 0;
   char** argv;
   ros::init(argc, argv, "ros_server");
   
   ros::NodeHandle nodeHandle;

   ros::Subscriber sub = nodeHandle.subscribe("chatter2", 1000, chatterCallback);

   int sockfd,n;
   struct sockaddr_in servaddr,cliaddr;
   socklen_t len;
   unsigned char msg[1000];

   sockfd=socket(AF_INET,SOCK_DGRAM,0);

   struct timeval tv;

   tv.tv_sec = 1;  /* 30 Secs Timeout */
   tv.tv_usec = 0;  // Not init'ing this can cause strange errors

   setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));


   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
   servaddr.sin_port=htons(RX_PORT);
   bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr));





   Message m;
   m.data = malloc(sizeof(float));
   for (;;)
   {
      if (ros::ok())
      {
         len = sizeof(cliaddr);
         n = recvfrom(sockfd,msg,1000,0,(struct sockaddr *)&cliaddr,&len);
         msg[n] = 0;

         int topicLen = (int)msg[0];
         char topicName[topicLen+1];
         memcpy(topicName, &msg[1], topicLen);
         topicName[topicLen] = '\0';
         //printf("%d %s: %s\n", topicLen, &msg[1], topicName);
         if (!strcmp(topicName, "advertise"))
         {
            std::string key((const char*)&msg[2+topicLen]);
            
            if (publishers.find(key) == publishers.end() ) 
            {
               // not found
               std::cout << "topic:" << key << " does not exist. Adding new publisher..." << std::endl;
               MessageType type = (MessageType)msg[topicLen+1];
               if(type == RCL_MSG_TYPE_FLOAT)
               {
                  printf("%s: %s type:%d -> float\n", topicName, &msg[2+topicLen], (int)type);
                  publishers[key] = std::make_pair(nodeHandle.advertise<std_msgs::Float32>(key, 1000), RCL_MSG_TYPE_FLOAT);
               }
               else if(type == RCL_MSG_TYPE_UINT32)
               {
                  printf("%s: %s type:%d -> int\n", topicName, &msg[2+topicLen], (int)type);
                  publishers[key] = std::make_pair(nodeHandle.advertise<std_msgs::Int32>(key, 1000), RCL_MSG_TYPE_UINT32);
               }
            } 
            else 
            {
               // found
               std::cout << "topic:" << key << " already exists." << std::endl;
            }


         }
         else
         {

            /*int offset = deserialize_int32(&m, &msg[1+topicLen]);
            printf("%s: %d\n", topicName, *((int*)(m.data)));*/
            std::string key((const char*)topicName);
            if (publishers.find(key) != publishers.end() ) // if a publisher with the topic already exists 
            {
               MessageType type = publishers[key].second;
               if(type == RCL_MSG_TYPE_FLOAT)
               {
                  int offset = deserialize_float32(&m, &msg[1+topicLen]);
                  float data = *((float*)(m.data));
                  printf("%s: %f\n", topicName, data);

                  std_msgs::Float32 message;
                  message.data = data;
                  publishers[key].first.publish(message);
               }
               else if (type == RCL_MSG_TYPE_UINT32)
               {
                  int offset = deserialize_int32(&m, &msg[1+topicLen]);
                  int data = *((int*)(m.data));
                  printf("%s: %d\n", topicName, data);

                  std_msgs::Int32 message;
                  message.data = data;
                  publishers[key].first.publish(message);
                  
               }
               
            }

         }
      }
      else
         exit(0);
   
      ros::spinOnce();

   }


}
#define TX_BUFFER_SIZE 24
void* TXThread(void* arg)
{


   int sockfd,n;
   struct sockaddr_in servaddr,cliaddr;
   char sendline[TX_BUFFER_SIZE];

   sockfd=socket(AF_INET,SOCK_DGRAM,0);

   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr=inet_addr(TX_IP_ADDR);
   servaddr.sin_port=htons(TX_PORT);


   Message m;
   m.data = malloc(sizeof(float));
   while(1)
	{
		for (int i=0; i<TX_BUFFER_SIZE; i++)
         sendline[TX_BUFFER_SIZE] = '\0';
      char* buffer = (char*)queue_dequeue((queue_t*)arg);
      memcpy(sendline, buffer, TX_QUEUE_MSG_SIZE);
      free(buffer);
      //sendto(sockfd,sendline,strlen(sendline),0, (struct sockaddr *)&servaddr,sizeof(servaddr));
      int chars = sendto(sockfd,sendline,TX_BUFFER_SIZE,0, (struct sockaddr *)&servaddr,sizeof(servaddr));
               int topicLen = (int)sendline[0];
         char topicName[topicLen+1];
         memcpy(topicName, &sendline[1], topicLen);
         topicName[topicLen] = '\0';
         int offset = deserialize_int32(&m, (unsigned char*)&sendline[1+topicLen]);
         int data = *((int*)(m.data));
         
      if (chars > -1)
      {
         printf("%s: %d, %d characters sent.\n", topicName, data, chars);

      }
      else
         printf("Could not send data! %d\n", data);




	}
}

int main(int argc, char** argv)
{
	
   pthread_t RXThreadHandle, TXThreadHandle;
m.data = malloc(sizeof(int));


	/*if(pthread_create(&RXThreadHandle, NULL, RXThread, NULL)) 
	{

		fprintf(stderr, "Error creating thread RX\n");
		return 1;
	}*/

	if(pthread_create(&TXThreadHandle, NULL, TXThread, &txQueue)) 
	{

		fprintf(stderr, "Error creating thread TX\n");
		return 1;
	}

   ros::init(argc, argv, "ros_server");
   
   ros::NodeHandle nodeHandle;

   ros::Subscriber sub = nodeHandle.subscribe("chatter2", 1000, chatterCallback);

   ros::spin();

	return 0;
}