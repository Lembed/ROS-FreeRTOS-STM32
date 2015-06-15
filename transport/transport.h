/*! \file transport.h
    \brief Transport layer.
*/

#ifndef ASW_OS_TRANSPORT_H_
#define ASW_OS_TRANSPORT_H_

/**
 * Used for logging a message on console that listens to UDP port LOG_LOCAL_PORT (currently 32005) on a remote PC.
 * msg is copied and sent to a queue and another task takes it from this queue and sends it to UDP buffer.
 */
void tr_log(const char* msg);

/**
 * Initializes transport data structures and parameters.
 */
void tr_init();


#endif /* ASW_OS_TRANSPORT_H_ */
