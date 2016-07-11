#ifndef RMW_XMLREQUEST_H_
#define RMW_XMLREQUEST_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "device_config.h"
#define XMLRPC_PORT 40000

static const char encoding_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                                      'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                                      'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                      'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                      'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
                                      'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                                      'w', 'x', 'y', 'z', '0', '1', '2', '3',
                                      '4', '5', '6', '7', '8', '9', '+', '/'
                                     };
static const int mod_table[] = {0, 2, 1};

class XMLRequest
{
protected:
	static char *base64_encode(const unsigned char *data, char* encoded_data, size_t input_length, size_t *output_length)
	{

		*output_length = 4 * ((input_length + 2) / 3);

		if (encoded_data == NULL) return NULL;

		for (int i = 0, j = 0; i < input_length;) {

			uint32_t octet_a = i < input_length ? (unsigned char)data[i++] : 0;
			uint32_t octet_b = i < input_length ? (unsigned char)data[i++] : 0;
			uint32_t octet_c = i < input_length ? (unsigned char)data[i++] : 0;

			uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

			encoded_data[j++] = encoding_table[(triple >> 3 * 6) & 0x3F];
			encoded_data[j++] = encoding_table[(triple >> 2 * 6) & 0x3F];
			encoded_data[j++] = encoding_table[(triple >> 1 * 6) & 0x3F];
			encoded_data[j++] = encoding_table[(triple >> 0 * 6) & 0x3F];
		}

		for (int i = 0; i < mod_table[input_length % 3]; i++)
			encoded_data[*output_length - 1 - i] = '=';

		return encoded_data;
	}

	static void createRequestHeader(const char* hostURI, int contentLength, char* data)
	{
		if (data != NULL) {
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
		if (data != NULL) {
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

		strcat(xml, "http://");
		strcat(xml, IP_ADDR);
		strcat(xml, ":");
		strcat(xml, portStr);
		strcat(xml, "</value> </param> </param> </params> </methodCall>");


		createRequestHeader(uri, strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}
};

class TopicRequest : public XMLRequest
{
	char base64[256];
	char connectionHeader[256];
public:
	TopicRequest(const char* methodName, const char* uri, const char* callerID, const char* topic, const char* md5sum, const char* msgType)
	{
		strcpy(xml, "<?xml version=\"1.0\"?> <methodCall> <methodName>");
		strcat(xml, methodName);
		strcat(xml, "</methodName>");
		strcat(xml, "<params><param><value>/");
		strcat(xml, callerID);
		strcat(xml, "</value></param><param><value>/");
		strcat(xml, topic);
		strcat(xml, "</value></param><param><value><array><data><value><array><data><value>UDPROS</value><value><base64>");
		uint32_t offset = 0;
		uint32_t len = 0;
		char str[50];

		sprintf(str, "callerid=/%s", callerID);
		len = (uint32_t)strlen(str);
		memcpy(connectionHeader + offset, &len, 4);
		offset += 4;
		memcpy(connectionHeader + offset, str, len);
		offset += len;


		sprintf(str, "md5sum=%s", md5sum);
		len = (uint32_t)strlen(str);
		memcpy(connectionHeader + offset, &len, 4);
		offset += 4;
		memcpy(connectionHeader + offset, str, len);
		//memcpy(connectionHeader+offset, "md5sum=992ce8a1687cec8c8bd883ec73ca41d1", len);
		offset += len;

		sprintf(str, "topic=/%s", topic);
		len = (uint32_t)strlen(str);
		memcpy(connectionHeader + offset, &len, 4);
		offset += 4;
		memcpy(connectionHeader + offset, str, len);
		offset += len;

		sprintf(str, "type=%s", msgType);
		len = (uint32_t)strlen(str);
		memcpy(connectionHeader + offset, &len, 4);
		offset += 4;
		memcpy(connectionHeader + offset, str, len);
		offset += len;

		//strcat(xml, "EgAAAGNhbGxlcmlkPS9saXN0ZW5lcicAAABtZDVzdW09OTkyY2U4YTE2ODdjZWM4YzhiZDg4M2VjNzNjYTQxZDEOAAAAdG9waWM9L2NoYXR0ZXIUAAAAdHlwZT1zdGRfbXNncy9TdHJpbmc=");
		unsigned int l;
		base64_encode((unsigned char*)connectionHeader, base64, offset, &l);

		strcat(xml, base64);

		strcat(xml, "</base64></value><value>");
		strcat(xml, IP_ADDR);
		strcat(xml, "</value><value><i4>44100</i4></value><value><i4>300</i4></value></data></array></value></data></array></value></param></params></methodCall>");

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

class PublisherUpdate : public XMLRequest
{
public:
	PublisherUpdate(const char* topic, const char* uri)
	{
		/*
		 * <?xml version='1.0'?>
		<methodCall>
		<methodName>publisherUpdate</methodName>
		<params>
		<param>
		<value><string>/master</string></value>
		</param>
		<param>
		<value><string>/rosout</string></value>
		</param>
		<param>
		<value><array><data>
		<value><string>http://SI-Z0M81:52656/</string></value>
		</data></array></value>
		</param>
		</params>
		</methodCall>
		 *
		 */
		strcpy(xml, "<?xml version='1.0'?><methodCall><methodName>publisherUpdate</methodName><params><param>");
		strcat(xml, "<value><string>/master</string></value></param><param><value><string>/");
		strcat(xml, topic);
		strcat(xml, "</string></value>");
		strcat(xml, "</param><param><value><array><data>");

		strcat(xml, "<value><string>");
		strcat(xml, uri);
		strcat(xml, "</string></value>");

		strcat(xml, "</data></array></value></param></params></methodCall>");

		createResponseHeader(strlen(xml), header);

		strcpy(data, header);
		strcat(data, xml);
	}

};

#endif /* RMW_XMLREQUEST_H_ */
