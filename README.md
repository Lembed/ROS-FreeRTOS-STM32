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

![](https://github.com/bosch-ros-pkg/stm32/blob/refactored/ROS.png)

## Notes:
- Node functions specified in nodes.h (a task table).
- Data transfer is implemented using UDPROS.
- Very simple XMLRPC handling (no XML parser is used) to make negotiations with ROS master on a PC.
- Messages are serialized using C++ headers generated with rosserial's message generator based on Python.

## How to setup
### Download and install CodeSourcery ARM Toolchain (Compiler)

Download the compiler from here:
https://sourcery.mentor.com/sgpp/lite/arm/portal/package9740/public/arm-none-eabi/arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2

Then install the compiler using below commands.
```
root@linux:/home/user/Downloads$ mkdir /opt/CodeSourcery
root@linux:/home/user/Downloads$ cd /opt/CodeSourcery
root@linux:/opt/CodeSourcery$ tar xvf /home/user/Downloads/arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
root@linux:/opt/CodeSourcery$ ls
arm-2011.09
root@linux:/opt/CodeSourcery$ echo PATH=\"$PATH:/opt/CodeSourcery/arm-2011.09/bin\" >> /etc/environment
root@linux:/opt/CodeSourcery$ source /etc/environment
root@linux:/opt/CodeSourcery$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
Copyright (C) 2011 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### Download and install STLink flash programmer software
```
user@linux:~$ mkdir src
user@linux:~$ cd src
user@linux:~/src$ git clone git://github.com/texane/stlink.git
user@linux:~/src$ cd stlink
user@linux:~/src/stlink$ ./autogen.sh
user@linux:~/src/stlink$ ./configure
user@linux:~/src/stlink$ make
```

```
root@linux:~$ mkdir /opt/stlink
root@linux:~$ cd /opt/stlink
root@linux:/opt/stlink$ cp /home/user/src/stlink/flash/flash ./st-flash
root@linux:/opt/stlink$ cp /home/user/src/stlink/gdbserver/st-util .
root@linux:/opt/stlink$ echo PATH=\"$PATH:/opt/stlink\" >> /etc/environment
root@linux:/opt/stlink$ source /etc/environment
```

### Compile the code
First, clone this repository to your workspace.
```
user@linux:/home/user/workspace$ git clone ...
user@linux:/home/user/workspace$ cd stm32
```

Then create a folder named bin and go into that folder.
```
user@linux:/home/user/workspace/stm32$ mkdir bin
user@linux:/home/user/workspace/stm32$ cd bin
```

Afterwards, call cmake command to generate a makefile.
```
user@linux:/home/user/workspace/stm32/bin$ cmake ..
```

For compiling the code, call the make command.
```
user@linux:/home/user/workspace/stm32/bin$ make
```

For flashing the code to an STM32F4Discovery, connect the board to computer's USB port and call the make flash command.
```
user@linux:/home/user/workspace/stm32/bin$ make flash
```