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

## Software Concept

The software concept is shown in the below image. Blue blocks represent FreeRTOS tasks.

![](https://github.com/bosch-ros-pkg/stm32/blob/refactored/doc/ROS.png)

Node functions are specified in the header file (node table) nodes.h, where developers need to type in the entry point of each ROS node on embedded system manually. Data transfer between PC and embedded system is realized using <a href="http://wiki.ros.org/ROS/UDPROS">UDPROS</a> over Ethernet. There is a very simple XMLRPC handling, which does not use any XML parser, to make negotiations with ROS master on a PC. ROS messages are serialized on embedded software using C++ headers generated with <a href="http://wiki.ros.org/rosserial">rosserial</a>'s message generator based on Python.

## How to Setup

A tutorial on how to set up the system is given <a href="https://github.com/bosch-ros-pkg/stm32/wiki">here</a>.