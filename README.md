# Real-Time ROS Nodes on STM32F4

## description
An embedded system supporting multiple ROS nodes that are able to communicate with one another using inter-process communication and with other ROS-compatible devices over a network. 
consists of an STM32F4Discovery board, the FreeRTOS os, embedded ROS middleware, embedded ROS client library, ROS nodes and sensor libraries application layer.

With this software, ROS developers do not have to know the complexities of the real-time operating system. 
ROS Client Library allows ROS developers to create Nodes, Publishers, Subscribers, and define ROS messages. 
This is why they can program their nodes as if they were writing code on a general-purpose computer.

The software includes two different scheduling schemes: priority and deadline. 
If deadline scheduling is enabled, application developers can assign deadlines to periodic tasks in order to make sure that they are completed before this deadline as long as the system is not overloaded. 


## contents

src/main 				: project main source code tree
src/apps 				: freertos tasks for each ros nodes and node drivers
src/board 				: bsp and device drivers for stm32f4discovery board
src/device      		: device driver support
src/core/os/kernel 		: freertos source code tree
src/core/os/syslibs 	: math and io support librarys
src/core/rcl 			: middleware for ros runable in embedded environment
src/core/rmw 			: xml rpc server and client built on lwip udp
src/core/lwip 			: lwip and port files for stm32f4
scripts  				: cmake script file to generate the project makefile
tutorials 				: ros node tutorials can be executed in embedded environment


## warning: 
This project is a experimental software,  a fork of [stm32](https://github.com/bosch-ros-pkg/stm32)

## license
Much of the code and documentation enclosed is copyright by the Free Software Foundation, Inc.
See the file copyright/license in the various directories
