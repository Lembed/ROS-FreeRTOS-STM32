#include <Subscriber.h>
namespace ros {

int Subscriber_::lastSubscriberIndex = -1;

Subscriber_* Subscribers_[MAX_SUBSCRIBERS];
Subscriber_** Subscriber_::list = Subscribers_;


} /* namespace ros */
