#ifndef RMW_XMLREQUEST_H_
#define RMW_XMLREQUEST_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define XMLRPC_PORT 40000

class XMLRequest
{
protected:
	static void createRequestHeader(const char* hostURI, int contentLength, char* data)
	{
		if (data != NULL)
		{
			strcpy(data, "POST / HTTP/1.1\nUser-Agent: curl/7.35.0\nHost: ");
			strcat(data, hostURI);
			strcat(data, "\\Accept: */*\nContent-Length: ");
			char contentLen[16];
			sprintf(contentLen, "%d", contentLength);
			strcat(data, contentLen);
			strcat(data, "\nContent-Type: application/x-www-form-urlencoded\n\n");
		}
	}
	static void createResponseHeader(int contentLength, char* data)
	{
		if (data != NULL)
		{
    		strcpy(data, "HTTP/1.0 200 OK\nServer: BaseHTTP/0.3 Python/2.7.6\n");
    		strcat(data, "Date: Sat, 01 January 1970 00:00:00 GMT\nContent-type: text/xml\nContent-length: ");
            char contentLen[16];
    		sprintf(contentLen, "%d", contentLength);
    		strcat(data, contentLen);
    		strcat(data, "\n\n");
		}
	}

	char data[1500];
	char header[256];
	char xml[1024];
public:
	const char* getData()
	{
		return data;
	}
};

class RegisterRequest : public XMLRequest
{
public:
	RegisterRequest(const char* methodName, const char* uri, const char* callerID, const char* topic, const char* msgType)
	{
		strcpy(xml, "<?xml version=\"1.0\"?> <methodCall> <methodName>");
		strcat(xml, methodName);
		strcat(xml, "</methodName> <params> <param> <value>/");
		strcat(xml, callerID);
		strcat(xml, "</value> </param> <param> <value>");
		strcat(xml, topic);
		strcat(xml, "</value> <param> <value>");
		strcat(xml, msgType);
		strcat(xml, "</value> </param> <param> <value>");
		char portStr[6];
		sprintf(portStr, "%d", XMLRPC_PORT);
		strcat(xml, "http://10.3.84.99:");
		strcat(xml, portStr);
		strcat(xml, "</value> </param> </param> </params> </methodCall>");


		createRequestHeader(uri, strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}
};

class TopicRequest : public XMLRequest
{
public:
	TopicRequest(const char* methodName, const char* uri, const char* callerID, const char* topic)
	{
		strcpy(xml, "<?xml version=\"1.0\"?> <methodCall> <methodName>");
		strcat(xml, methodName);
		strcat(xml, "</methodName>");
		strcat(xml, "<params><param><value>/");
		strcat(xml, callerID);
		strcat(xml, "</value></param><param><value>/");
		strcat(xml, topic);
		strcat(xml, "</value></param><param><value><array><data><value><array><data><value>UDPROS</value><value><base64>EgAAAGNhbGxlcmlkPS9saXN0ZW5lcicAAABtZDVzdW09OTkyY2U4YTE2ODdjZWM4YzhiZDg4");
		strcat(xml, "M2VjNzNjYTQxZDEOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=</base64></value><value>10.3.84.99</value><value><i4>44100</i4></value><value><i4>1500</i4></value></data></array></value></data></array></value></param></params></methodCall>");

		createRequestHeader(uri, strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}
};

class TopicResponse : public XMLRequest
{
public:
	TopicResponse(const char* localIP, const uint16_t& localPort, const uint32_t& connectionID)
	{
		char tmp[15];
		strcpy(xml, "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><i4>1</i4></value>");
		strcat(xml, "<value></value><value><array><data><value>UDPROS</value><value>");
		strcat(xml, localIP);
		strcat(xml, "</value><value><i4>");
		sprintf(tmp, "%d", localPort);
		strcat(xml, tmp);
		strcat(xml, "</i4>");
		strcat(xml, "</value><value><i4>");
		sprintf(tmp, "%d", connectionID);
		strcat(xml, tmp);
		strcat(xml, "</i4>");
		strcat(xml, "</value><value><i4>1500</i4></value><value><base64>");
		// TODO: Build the base64encoded data (Connection Header) dynamically.
		strcat(xml, "EAAAAGNhbGxlcmlkPS90YWxrZXInAAAAbWQ1c3VtPTk5MmNlOGExNjg3Y2VjOGM4YmQ4ODNlYzczY2E0MWQxHwAAAG1lc3NhZ2VfZGVmaW5pdGlvbj1zdHJpbmcgZGF0YQoOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=");
		strcat(xml, "</base64></value></data></array></value></data></array></value></param></params></methodResponse>");

		createResponseHeader(strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}

};

#endif /* RMW_XMLREQUEST_H_ */
