# ROS on STM32

### Introduction

This software introduces ROS on a bare-metal operating system, which is FreeRTOS in this example. Here is briefly what this software does:
- Easy to create nodes with subscriber callbacks and publisher calls.
- API looks very similar to roscpp.
- Deterministic behavior using bare-metal OS constructs.
- Inter-node communication.

### Description
Software for distributing nodes in a multi-node ROS system on STM32 microcontroller. It consists of a multi-tasking operating system (FreeRTOS), ROS middleware, ROS client library, and application tasks. Currently, only subscribing and publishing are supported. There is an XMLRPC server for communicating with the ROS master node. UDPReceive and UDPSend tasks are used to receive/send packets over UDP where this software only supports the UDPROS protocol.


|           Layers          |
| ---------------------------------| 
|           application           |
|        ROS Client Library       |
|         ROS Middleware          |
|            XMLRPC+UDP           |
|             FreeRTOS            |
|        Hardware (STM32F4)       |



Similar to the ROS 2.0 middleware implementation, there are TopicReaders and TopicWriters as if DDS was being used. This abstraction will make it easier to replace the XMLRPC+UDP mechanism by DDS in the future.

In the below image, it is shown how TopicReaders/Writers, UDPReceive/Send tasks, and Publishers/Subscribers relate to one another.

![](https://github.com/bosch-ros-pkg/stm32/blob/eclipse/ROS.png)

## Notes:
- Node functions specified in application_tasks.h (a task table).
- Data transfer is implemented using UDP.
- Very simple XMLRPC handling (no XML parser is used).
- Messages are serialized using C++ headers generated with rosserial's message generator based on Python.

### TODO:
- Test the real-time behavior.
- Make sure UDP receiving does not block UDP sending or vice versa. (Check the maximum number of concurrent UDP/TCP connections)
- Real-world example.