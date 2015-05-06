# ROS on STM32

### Introduction

This software introduces ROS on a bare-metal operating system, which is FreeRTOS in this example. Here is briefly what this software does:
- Easy to create nodes with subscriber callbacks and publisher calls.
- API looks very similar to roscpp.
- Deterministic behavior using bare-metal OS constructs.
- Inter-node communication.

### Description
Software for distributing nodes in a multi-node ROS system on FreeRTOS. Currently, this code is similar to the rosserial approach since it sends its messages to a rosserver (on a desktop computer), which transmits incoming messages to the corresponding ROS topics. The communication is realized using UDP, where a transmission rate around 200 Hz can be achieved. This approach is different from rosserial in that we have multiple nodes. 

### TODO:
- Create C++ classes for RCL instead of using C functions.
- Instead of having a rosserver, try sending XMLRPC requests and interact with the ROS master on desktop directly.

## Notes:
- Node functions specified in application_tasks.h (a task table).
- Data transfer is implemented using UDP.