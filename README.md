# ROS on STM32

## Description
An embedded system supporting multiple ROS nodes that are able to communicate with one another using inter-process communication and with other ROS-compatible devices over a network. This system consists of an STM32F4Discovery board, the FreeRTOS operating system, an embedded ROS middleware, and an embedded ROS client library. Application layer is the layer for ROS nodes and sensor libraries.


|           Layers          |
| ---------------------------------| 
|           application           |
|        ROS Client Library       |
|         ROS Middleware          |
|            XMLRPC+UDP           |
|             FreeRTOS            |
|        Hardware (STM32F4)       |


The software concept is shown in the below image. Blue blocks represent FreeRTOS tasks.

![](https://github.com/bosch-ros-pkg/stm32/blob/refactored/doc/ROS.png)

## Notes:
- Node functions specified in nodes.h (a task table).
- Data transfer is implemented using UDPROS.
- Very simple XMLRPC handling (no XML parser is used) to make negotiations with ROS master on a PC.
- Messages are serialized using C++ headers generated with rosserial's message generator based on Python.

## How to setup
https://github.com/bosch-ros-pkg/stm32/wiki/How-to-Setup