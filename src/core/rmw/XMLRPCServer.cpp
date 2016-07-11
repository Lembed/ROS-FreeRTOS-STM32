#include "XMLRPCServer.h"

#include "tcp.h"
#include <lwip/sockets.h>
#include "lwip/ip_addr.h"
#include "tcpip.h"
#include "api.h"
#include "device_config.h"

extern "C"
{
#include "ros.h"
}

#include "XMLRequest.h"


#define TOPIC_COUNT 20
#define MAX_TOPIC_LEN 48


#include "netconf.h"

bool XMLRPCServer::isUDPReceiveTaskCreated = false;

#define TCP_DATA_SIZE 1200
class HTTPClient
{
private:
    struct TCPData {
        uint16_t serverPort;
        uint32_t serverIP;
        char data[TCP_DATA_SIZE];
        void(*receiveCallback)(const void* obj, const char* data);
        void* obj;
    };

    static void tcptask(void* arg)
    {
        static uint16_t port = 30000;
        HTTPClient* self = (HTTPClient*) arg;
        TCPData tcpData;
        for (;;) {
            if (xQueueReceive(self->qHandle, &tcpData, 100)) {
                port++;
                struct netconn *conn = netconn_new(NETCONN_TCP);
                err_t err;
                if (conn != NULL) {
                    // Bind connection to the specified number
                    os_printf("Binding port %d\n", port);
                    err = netconn_bind(conn, NULL, port);

                    if (err == ERR_OK) {
                        struct ip_addr ip;
                        ip.addr = tcpData.serverIP;
                        os_printf("Connecting port %d\n", tcpData.serverPort);
                        err = netconn_connect (conn, &ip, tcpData.serverPort);

                        if (err == ERR_OK) {
                            os_printf("Writing data!\n");
                            netconn_write(conn, tcpData.data, TCP_DATA_SIZE, NETCONN_COPY);

                            struct netbuf *buf;
                            char *data;
                            u16_t len;
                            uint32_t offset = 0;
                            if ((buf = netconn_recv(conn)) != NULL) {
                                do {
                                    netbuf_data(buf, (void**)&data, &len);
                                    if (self->rxBuffer != NULL && data != NULL)
                                        memcpy(self->rxBuffer + offset, data, len);
                                    else
                                        os_printf("HTTPClient::tcpTask self->rxBuffer or data is NULL!\n");

                                    offset += len;
                                    os_printf("Netconn received %d bytes\n", len);


                                } while (netbuf_next(buf) >= 0);
                                self->onReceive(tcpData.receiveCallback, tcpData.obj, self->rxBuffer);
                                netbuf_delete(buf);
                            }
                        }
                    }

                }
                netconn_close (conn );
                netconn_delete (conn );
            }
        }

        vTaskDelete(NULL);
    }


    HTTPClient()
    {
        qHandle = xQueueCreate(4, sizeof(TCPData));
        xTaskCreate(tcptask, (const signed char*)"HTTPClient", 1500, this, tskIDLE_PRIORITY + 2, NULL);
    }

    static HTTPClient* _instance;

    void onConnected(uint16_t port)
    {
        os_printf("Connected, serverport:%d!\n", port);
    }
    void onReceive(void(*receiveCallback)(const void* obj, const char* data), void* obj, const char* data)
    {
        os_printf("Received %d bytes!\n", strlen(data));
        if (receiveCallback != NULL) {
            if (obj != NULL)
                receiveCallback(obj, data);
        }
    }
    void onSent()
    {
        os_printf("Sent!\n");
    }

    xQueueHandle qHandle;
    char rxBuffer[1024];

public:
    static HTTPClient *instance()
    {
        if (!_instance)
            _instance = new HTTPClient();
        return _instance;
    }
    void sendData(const char* data, uint16_t port, void(*receiveCallback)(const void* obj, const char* data) = NULL, void* obj = NULL)
    {
        TCPData tcpData;
        if (data != NULL) {
            memcpy(tcpData.data, data, TCP_DATA_SIZE);
        } else
            os_printf("HTTPClient::sendData data is NULL!\n");
        tcpData.serverIP = inet_addr(ROS_MASTER_IP);
        tcpData.serverPort = port;
        tcpData.receiveCallback = receiveCallback;
        tcpData.obj = obj;

        if (xQueueSend(qHandle, &tcpData, 0))
            os_printf("Enqueueing data!\n");
        else
            os_printf("Queue is full!\n");
    }

};

HTTPClient* HTTPClient::_instance = NULL;

#define HTTP_SERVER_BUFFER_SIZE 1500
class HTTPServer
{
private:
    static void close_conn(struct tcp_pcb *pcb)
    {
        tcp_arg(pcb, NULL);
        tcp_sent(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_close(pcb);
    }
    static err_t data_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
    {
        HTTPServer* self = (HTTPServer*) arg;
        self->onSendAcknowledged();
    }
    static err_t echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
    {
        os_printf("Echo recv!\n");
        HTTPServer* self = (HTTPServer*) arg;
        int i;
        int len;
        char *pc;
        if (err == ERR_OK && p != NULL) {
            // Inform TCP that we have taken the data.
            tcp_recved(pcb, p->tot_len);
            os_printf("TCP recv no error!\n");

            //pointer to the pay load
            pc = (char *)p->payload;

            //size of the pay load
            len = p->tot_len;

            uint16_t bufferLength = 1500;
            char buffer[bufferLength];
            self->onReceive(pc);
            self->receiveCallback(pc, buffer);

            pbuf_free(p);

            err = tcp_write(pcb, buffer, bufferLength, 0);
            tcp_sent(pcb, data_sent);

        }
        if (err == ERR_OK && p == NULL) {
            close_conn(pcb);
        }
        return ERR_OK;
    }

    static err_t echo_accept(void *arg, struct tcp_pcb *pcb, err_t err)
    {
        HTTPServer* self = (HTTPServer*) arg;
        self->onAccept();
        tcp_setprio(pcb, TCP_PRIO_MIN);
        tcp_recv(pcb, echo_recv);
        tcp_err(pcb, NULL); //Don't care about error here
        tcp_poll(pcb, NULL, 4); //No polling here
        return ERR_OK;
    }

    static void tcptask(void* arg)
    {
        struct tcp_pcb *pcb = tcp_new();
        HTTPServer* self = (HTTPServer*) arg;
        tcp_arg(pcb, arg);
        tcp_bind(pcb, IP_ADDR_ANY, self->port);

        os_printf("TCP listen!\n");
        pcb = tcp_listen(pcb);
        tcp_accept(pcb, echo_accept);

        // Keep task alive.
        for (;;) {
            vTaskDelay(60000);
        }
    }
    void onAccept()
    {
        os_printf("Accept, port:%d!\n", port);
        uint16_t remainingHeap = xPortGetFreeHeapSize();
        os_printf("on accept: remaining heap: %dB!\n", remainingHeap / 10);
    }
    void onReceive(const char* data)
    {
        os_printf("Received %d bytes!\n", strlen(data));
    }
    void onSendAcknowledged()
    {
        os_printf("Sent data!\n");
    }

    uint16_t port;
    void(*receiveCallback)(const char* data, char* buffer);
public:
    HTTPServer(const char* taskName, uint16_t port, void(*receiveCallback)(const char* data, char* buffer) = NULL)
    {
        this->port = port;
        this->receiveCallback = receiveCallback;
        xTaskCreate(tcptask, (const signed char*)taskName, 1600, this, tskIDLE_PRIORITY + 2, NULL);
    }

};


extern "C" void ICMP_callback(struct pbuf *p, struct netif *inp)
{
    if (*(((u8_t *)p->payload) + 20) == 3) { // Port unreachable
        uint16_t port = ntohs(*(((u16_t *)(p->payload + 50))));
        os_printf("icmp_input: type: %d port: %d\n", *(((u8_t *)p->payload) + 20), ntohs(*(((u16_t *)(p->payload + 50)))));
        //os_printf("icmp_input: %08x %08x\n", *(((u32_t *)p->payload)), *(((u32_t *)p->payload)+1));
        TopicWriter* tw = XMLRPCServer::getTopicWriter(port);
        tw->deleteConnection(port);
    }
}



#define MAX_TOPIC_WRITERS 10
TopicWriter* topicWriters[MAX_TOPIC_WRITERS];

#define MAX_TOPIC_READERS 10
TopicReader* topicReaders[MAX_TOPIC_READERS];
bool pin3 = true;
#include "wiring.h"
void XMLRPCServer::UDPSend(void* params)
{
    UDPHandler* uh = UDPHandler::instance();
    struct netconn* conn = netconn_new( NETCONN_UDP );
    netconn_bind(conn, IP_ADDR_ANY, UDP_LOCAL_PORT);
    static uint8_t counter = 1;

    pinMode(GPIO_PD11, OUTPUT);
    UDPMessage msg;
    struct ip_addr ip;
    ip.addr = inet_addr(ROS_MASTER_IP);
    for (;;) {
        //digitalWrite(GPIO_PD11, HIGH);
        digitalWrite(GPIO_PD11, pin3);
        uh->dequeueMessage(&msg);
        //digitalWrite(GPIO_PD11, LOW);

        // inter-node communication
        TopicReader* tr = getTopicReader(msg.topic);
        if (tr != NULL) {
            tr->enqueueMessage(msg.data);
        }

        TopicWriter* tw = getTopicWriter(msg.topic);
        if (tw != NULL) {

            UDPConnection* const* connections = tw->getConnections();
            if (connections != NULL) {
                for (int i = 0; i < MAX_UDP_CONNECTIONS; i++) {
                    const UDPConnection* connection = connections[i];
                    if (connection && connection->isValid()) {
                        err_t err = netconn_connect(conn, &ip, connection->getPort());
                        //os_printf("1Port: %d LWIP Error:%d\n", endpoint.port, err);

                        os_printf("Connecting %s:%d, err:%d\n", ip, connection->getPort(), err);
                        struct netbuf *buf = netbuf_new();
                        char msgHeader[8];
                        uint32_t connectionID = connection->getID();
                        memcpy(&msgHeader[0], &connectionID, sizeof(uint32_t));
                        msgHeader[4] = 0;
                        msgHeader[5] = counter++;
                        msgHeader[6] = 0x01;
                        msgHeader[7] = 0;
                        uint32_t msgLen = *((uint32_t*) msg.data) + 4;
                        void* data = netbuf_alloc(buf, msgLen + sizeof(msgHeader)); // Also deallocated with netbuf_delete(buf)
                        if (data != NULL) {
                            memcpy (data, msgHeader, sizeof(msgHeader));
                            memcpy (data + sizeof(msgHeader), msg.data, msgLen);
                        } else {
                            os_printf("XMLRPCServer::UDPSend data is NULL!\n");
                        }

                        err = netconn_send(conn, buf);
                        //os_printf("2Port: %d LWIP Error:%d\n", endpoint.port, err);

                        netbuf_delete(buf);
                    }
                }
            }
        }
        pin3 = !pin3;
    }
}

TopicWriter* XMLRPCServer::getTopicWriter(const uint16_t port)
{
    for (uint16_t i = 0; i < MAX_TOPIC_WRITERS; i++) {
        if (topicWriters[i] != NULL) {
            TopicWriter* tw = topicWriters[i];
            if (tw) {
                UDPConnection** connections = (UDPConnection**)tw->getConnections();
                for (uint16_t i = 0; i < MAX_UDP_CONNECTIONS; i++) {
                    if (connections[i] != NULL && connections[i]->getPort() != 0) {
                        return tw;
                    }
                }
            }
        }
    }
    return NULL;
}

TopicWriter* XMLRPCServer::getTopicWriter(const char* topic)
{
    for (uint16_t i = 0; i < MAX_TOPIC_WRITERS; i++) {
        if (topicWriters[i] != NULL) {
            TopicWriter* tw = topicWriters[i];
            if (!strcmp(tw->getTopic(), topic)) {
                return tw;
            }
        }
    }
    return NULL;
}

TopicReader* XMLRPCServer::getTopicReader(const char* topic)
{
    for (uint16_t i = 0; i < MAX_TOPIC_READERS; i++) {
        if (topicReaders[i] != NULL) {
            TopicReader* tr = topicReaders[i];
            if (!strcmp(tr->getTopic(), topic)) {
                return tr;
            }
        }
    }
    return NULL;
}

TopicReader* XMLRPCServer::getTopicReader(const uint32_t connectionID)
{
    for (uint16_t i = 0; i < MAX_TOPIC_READERS; i++) {
        if (topicReaders[i] != NULL) {

            TopicReader* tr = topicReaders[i];
            if (tr->getConnectionID() == connectionID) {
                return tr;
            }

        }
    }
    return NULL;
}

void XMLRPCServer::XMLRPCServerReceiveCallback(const char* data, char* buffer)
{
    os_printf("Receive callback, buffer addr: %08x!\n", buffer);

    char methodName[48];
    {
        char* pos = strstr((char*)data, "<methodName>");
        char* pos2 = strstr((char*)data, "</methodName>");

        if (pos2 > pos) {
            strncpy (methodName, pos + 12, pos2 - pos - 12);
            methodName[pos2 - pos - 12] = 0;
        }
    }

    os_printf("name:%s\n", methodName);
    os_printf("Strlen:%d\n", strlen(data));

    if (!strcmp(methodName, "requestTopic")) {

        char* pos = strstr(data, "<i4>");
        char* pos2 = strstr(data, "</i4>");
        os_printf("pos:%d, pos2:%d\n", pos, pos2);

        if (pos < pos2) {
            char portStr[pos2 - pos - 3];
            strncpy (portStr, pos + 4, pos2 - pos - 4);
            portStr[pos2 - pos - 4] = 0;
            uint16_t port = atoi(portStr);
            os_printf("Port: %d\n", port);

            char* pos3 = strstr((char*)data, "</value></param><param><value>/");
            char* pos4;
            int len = strlen("</value></param><param><value>/");

            if (pos3) {
                pos4 = strstr((char*)pos3 + len, "</value>");
                if (pos4 > pos3) {
                    char topic[pos4 - pos3 - len + 1];
                    strncpy (topic, pos3 + len, pos4 - pos3 - len);
                    topic[pos4 - pos3 - len] = 0;
                    os_printf("topic: %s\n", topic);

                    // TODO: Move UDPConnection to registerPublishers. Then extract topic name from data. Afterwards, find the corresponding connection.
                    TopicWriter* tw = getTopicWriter(topic);
                    if (tw != NULL) {
                        UDPConnection* connection = tw->getConnection(port);
                        if (connection != NULL) {
                            os_printf("Connection ID: %d\n", connection->getID());
                            XMLRequest* response = new TopicResponse(IP_ADDR, UDP_LOCAL_PORT, connection->getID());
                            strcpy(buffer, response->getData());
                        }
                    }
                }
            }
        }
    } else if (!strcmp(methodName, "publisherUpdate")) {
        os_printf("Publisher Update!\n");
        char* pos = strstr(data, "<value><string>/master</string></value>");
        if (pos != 0) {
            char topic[MAX_TOPIC_LEN];
            while (1) {
                char* pos2 = strstr((char*)pos, "<value><string>");
                char* pos3 = strstr((char*)pos2, "</string></value>");
                //os_printf("_pos:%d, _pos2:%d\n", pos2, pos3);
                if (pos2 == NULL || pos3 == NULL)
                    break;
                if (pos3 > pos2) {
                    int offset = strlen("<value><string>");
                    char uri[pos3 - pos2 - offset + 1];
                    strncpy (uri, pos2 + offset, pos3 - pos2 - offset);
                    uri[pos3 - pos2 - offset] = 0;

                    if (!strcmp(uri, "/master")) {
                    } else if (uri[0] == '/') {
                        strcpy(topic, &uri[1]);
                    } else {
                        uint16_t port;
                        char ip[32];
                        extractURI(uri, ip, &port);
                        if (strcmp(ip, IP_ADDR)) {
                            os_printf("Topic:%s URI: %s:::%s:::%d\n", topic, uri, ip, port);
                            TopicReader* tr = getTopicReader(topic);
                            if (tr != NULL) {
                                if (!strcmp(ip, "SI-Z0M81"))
                                    strcpy(ip, ROS_MASTER_IP);

                                tr->requestTopic(ip, port);
                            }
                        }
                    }
                }
                pos = pos3;
            }
        } else
            os_printf("pos is NULL\n");

    }
}


void XMLRPCServer::start()
{
    HTTPServer* server = new HTTPServer("HTTPServer", XMLRPC_PORT, XMLRPCServerReceiveCallback);
    xTaskCreate(UDPSend, (const signed char*)"UDPSend", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    isUDPReceiveTaskCreated = false;

    xTaskCreate(UDPreceive, (const signed char*)"UDPReceive", 256, NULL, tskIDLE_PRIORITY + 3, NULL);

}

TopicWriter* XMLRPCServer::registerPublisher(const char* callerID, const char* topic, const char* msgType)
{
    static uint16_t lastIndex = 0;
    TopicWriter* tw = new TopicWriter(callerID, topic, msgType);
    topicWriters[lastIndex++] = tw;
    return tw;
}

TopicReader* XMLRPCServer::registerSubscriber(const char* callerID, const char* topic, const char* md5sum, const char* msgType)
{
    static uint16_t lastIndex = 0;
    TopicReader* tr = new TopicReader(callerID, topic, md5sum, msgType);
    topicReaders[lastIndex++] = tr;
    return tr;
}

#define UDP_RECEIVE_PORT 44100

void XMLRPCServer::UDPreceive(void* params)
{
    struct netconn *conn;
    struct netbuf *buf;
    err_t err;

    // Initialize memory (in stack) for message.
    char message[60]; // TODO: Set its size according to UDPMessage.
    os_printf("Test!\n");
    conn = netconn_new(NETCONN_UDP);
    for (;;) {
        // Check if connection was created successfully.
        if (conn != NULL) {
            err = netconn_bind(conn, IP_ADDR_ANY, UDP_RECEIVE_PORT);

            // Check if we were able to bind to port.
            if (err == ERR_OK) {
                portTickType xLastWakeTime;
                // Initialize the xLastWakeTime variable with the current time.
                xLastWakeTime = xTaskGetTickCount();

                // Start periodic loop.
                while (1) {
                    buf = netconn_recv(conn);
                    if (buf != NULL) {
                        struct ip_addr* ip;
                        uint16_t port;
                        ip = buf->addr;
                        port = buf->port;
                        //if(ip != NULL)
                        //os_printf("Received from %d:%d!\n", ip->addr, port);
                        // Copy received data into message.
                        uint16_t len = netbuf_len(buf);
                        if (len > 15) {
                            netbuf_copy (buf, &message, len);
                            uint32_t connectionID = *((uint32_t*)&message[0]);
                            TopicReader* tr = getTopicReader(connectionID);
                            if (tr != NULL) {
                                tr->enqueueMessage(&message[8]);
                                //os_printf("ConnectionID: %d, topic:%s\n", connectionID, tr->getTopic());
                            }
                        }
                        // Deallocate previously created memory.
                        netbuf_delete(buf);
                    }
                    // Use delay until to guarantee periodic execution of each loop iteration.
                    else {
                        os_printf("buf = NULL!\n");
                        vTaskDelayUntil(&xLastWakeTime, 30);
                    }
                }
            } else {
                os_printf("cannot bind netconn\n");
            }
        } else {
            os_printf("cannot create new UDP netconn\n");
            conn = netconn_new(NETCONN_UDP);
        }
        // If connection failed, wait for 50 ms before retrying.
        vTaskDelay(50);
    }
}

void XMLRPCServer::extractURI(const char* uri, char* ip, uint16_t* port)
{
    char* p = strstr(uri, "://");
    if (p != NULL) {
        char* pos = strstr(p + 3, ":");
        if (pos != NULL && ip != NULL) {
            memcpy(ip, p + 3, pos - p - 3);
            ip[pos - p - 3] = 0;

            char portStr[6];
            strcpy(portStr, pos + 1);
            *port = atoi(portStr);
        } else
            os_printf("XMLRPCServer::extractURI pos or ip is NULL!\n");
    }
}

void XMLRPCServer::sendRequest(const char* data, uint16_t port, void(*receiveCallback)(const void* obj, const char* data), void* obj)
{
    HTTPClient::instance()->sendData(data, port, receiveCallback, obj);
}
