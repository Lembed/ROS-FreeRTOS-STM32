# Program Real-Time ROS Nodes on STM32

## Description
An embedded system supporting multiple ROS nodes that are able to communicate with one another using inter-process communication and with other ROS-compatible devices over a network. This system consists of an STM32F4Discovery board, the FreeRTOS operating system, an embedded ROS middleware, and an embedded ROS client library. Application layer is the layer for ROS nodes and sensor libraries.

With this software, ROS developers do not have to know the complexities of the real-time operating system. ROS Client Library allows ROS developers to create Nodes, Publishers, Subscribers, and define ROS messages. This is why they can program their nodes as if they were writing code on a general-purpose computer.

The software includes two different scheduling schemes: priority and deadline. If deadline scheduling is enabled, application developers can assign deadlines to periodic tasks in order to make sure that they are completed before this deadline as long as the system is not overloaded. 

WARNING: Completely experimental software!  a fork of [stm32](https://github.com/bosch-ros-pkg/stm32)
