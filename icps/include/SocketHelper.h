/*
 * SocketHelper.h
 *
 *  Created on: Jan 25, 2018
 *      Author: retry
 */

#ifndef SOCKETHELPER_H_
#define SOCKETHELPER_H_

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>

#include "TLVHelper.h"
#include "ROSHelper.h"

#define MaxNumberOfAttemptInSocketRead 20
#define MinSleepTimeBetweenAttempInSocketRead 1	//10us

using namespace std;

class SocketHelper {
public:
	static void createServerSockAndWaitForConnection(string _ip, int _port, bool _useIP, int* _serversocketFD, void* (*_acceptClientRequest)(void*)) {
		int status;
		*_serversocketFD = socket(AF_INET, SOCK_STREAM, 0);
		if (*_serversocketFD < 0){
			printf("Fail to create server socket with error: %s\n", strerror(errno));
			exit(1);
		}

		int enable = 1;
		if (setsockopt(*_serversocketFD, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0){
			printf("setsockopt(SO_REUSEADDR) failed\n");
		}

		struct sockaddr_in serverAddress;
		bzero((char *) &serverAddress, sizeof(serverAddress));
		serverAddress.sin_family = AF_INET;

		serverAddress.sin_addr.s_addr = _useIP ? inet_addr(_ip.c_str()) : INADDR_ANY;
		serverAddress.sin_port = htons(_port);
		status = bind(*_serversocketFD, (struct sockaddr *) &serverAddress, sizeof(serverAddress));
		if (status < 0){
			printf("Fail to bind server socket with error: %s\n", strerror(errno));
			exit(1);
		}
		listen(*_serversocketFD, 5);

	    struct sockaddr_in clientAddress;
	    socklen_t clilen=sizeof(clientAddress);
	    pthread_t newthread;
	    char clientAddressString[INET_ADDRSTRLEN];
	    int *clientsockfd;

	    printf("Waiting for connection...\n");
	    while(true)
	    {
	    	clientsockfd = (int*) malloc(sizeof(int));
	    	*clientsockfd = accept(*_serversocketFD, (struct sockaddr *) &clientAddress, &clilen);

	    	time_t now = time(0);
	    	char* dt = ctime(&now);

	    	if(*clientsockfd < 0){
	        	printf("Accept function fail with error: %s (%d), at %s\n", strerror(errno), errno, dt);
			if(errno == EBADF){
				break;
			}else{
	        		continue;
			}
	        }else{
	        	inet_ntop( AF_INET, &clientAddress.sin_addr, clientAddressString, INET_ADDRSTRLEN);
	        	status = pthread_create(&newthread, NULL, *_acceptClientRequest, clientsockfd);
	        	if(status < 0){
	            	printf("Fail to create thread with error: %s! Close sock and continue\n", strerror(status));
	            	close(*clientsockfd);
	            	continue;
	            }else{
	            	printf("Accept client request from ip: %s:%d (socket id: %d, thread id: %ld) at %s\n", clientAddressString, clientAddress.sin_port, *clientsockfd, newthread, dt);
	            }
	        }
	    }
	}

	static int createClientSockAndConnect(string _ip, int _port) {
		int clientSockFD, status;
		struct sockaddr_in addr;

		clientSockFD = socket(AF_INET, SOCK_STREAM, 0);
		addr.sin_family = AF_INET;
		addr.sin_port = htons(_port);
		inet_aton(_ip.c_str(), &addr.sin_addr);

		status = connect(clientSockFD, (sockaddr*) &addr, sizeof(addr));
		if(status < 0){
			ERRMSG("Create socket error: %s\n", strerror(errno));
		}

		return clientSockFD;
	}

	static void closeSocketAndPrintStatus(int _socketFD) {
		int status = close(_socketFD);
		if(status < 0){
			ERRMSG("Close socket error: %s\n", strerror(errno));
		}
	}

	static void shutdownSocketAndPrintStatus(int _socketFD) {
		int status = shutdown(_socketFD, SHUT_RDWR);
		if(status < 0){
			ERRMSG("Shutdown socket error: %s\n", strerror(errno));
		}
		status = close(_socketFD);
		if(status < 0){
			ERRMSG("Close socket error: %s\n", strerror(errno));
		}
	}

	static bool isClose(int _sock) {
		INFMSG("polling");
		struct pollfd fds[1];
		fds[0].fd = _sock;
		fds[0].events = 0;
		fds[0].revents = 0;

		int ready = poll(fds, 1, 0);
//		INFMSG("polled %d %d", ready, fds[0].revents);
		return (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) ? true : false;
	}

	static char* receiveTLV(int _sock, bool isBlocking=true) {
		int numByteReceived = 0;
		char TLData[TL_HEADER_SIZE];
		int firstType, firstLength;
		char* ret;
		char* valueHexString;

		numByteReceived = receiveAppendUntilGetExpectedLength(_sock, TLData, TL_HEADER_SIZE, isBlocking);
		if(numByteReceived < TL_HEADER_SIZE){
			if(numByteReceived == 0 && !isBlocking){
				return NULL;
			}else{
				ERRMSG("Cannot read first type and length value! Close sock and return");
				close(_sock);
				return NULL;
			}
		}

		firstType = TLVHelper::getTLVTypeField(TLData);
		firstLength = TLVHelper::getTLVLengthField(TLData);
		if(firstLength < 0){
			ERRMSG("First length value is negative (%d)! Close sock and return\n", firstLength);
			close(_sock);
			return NULL;
		}
		int TLVLength = TL_HEADER_SIZE + firstLength;
		ret = (char*) malloc(TLVLength*sizeof(char));
		memcpy(ret, TLData, TL_HEADER_SIZE);
		numByteReceived = receiveAppendUntilGetExpectedLength(_sock, ret+TL_HEADER_SIZE, firstLength, isBlocking);
		if(numByteReceived < firstLength){
			ERRMSG("Read data size (%d) is different from expected data size (%d)! Close sock and return\n", numByteReceived, firstLength);
			free(valueHexString);
			close(_sock);
			return NULL;
		}

		return ret;
	}

	static int receiveAppendUntilGetExpectedLength(int _sock, char* _byteArray, unsigned int _expectedLength, bool isBlocking=true) {
		int numByteReceived = 0, numAdditionalByteReceived = 0, numReceiveTime = 0;
		int numberOfAttempt = MaxNumberOfAttemptInSocketRead;
		int sleepTime = MinSleepTimeBetweenAttempInSocketRead;

		while(_expectedLength-numByteReceived > 0){
			int flag = !isBlocking && numByteReceived == 0 ? MSG_DONTWAIT : 0;
			numAdditionalByteReceived = recv(_sock, _byteArray+numByteReceived, _expectedLength-numByteReceived, flag);
			if(numAdditionalByteReceived == -1 && numByteReceived != _expectedLength){
				ERRMSG("Fail to receive with error: %d (%s) in socket fd %d! Total read size %d/%d!\n", errno, strerror(errno), _sock, numByteReceived, _expectedLength);
				break;
			}else if(numAdditionalByteReceived == 0){
				if(flag > 0 && errno == EINPROGRESS){
					break;
				}else if(errno == EPIPE || errno == EAGAIN){
					ERRMSG("Fail to receive with error: %d (%s) in socket fd %d! Total read size %d/%d!\n", errno, strerror(errno), _sock, numByteReceived, _expectedLength);
					numByteReceived = -1;
					break;
				}else{
					ERRMSG("ERROR %d", errno);
				}
			}else{
				numByteReceived += numAdditionalByteReceived;
			}
		}

		return numByteReceived;
	}

	static int sendTLV(int _sock, const char* _TLV) {
		if(_TLV == NULL){
			ERRMSG("Null input");
			return -1;
		}

		int TLVSize = TLVHelper::getTLVSize(_TLV);
		int status = send(_sock, _TLV, TLVSize, 0);
		if(status <= 0){
			ERRMSG("Fail to send TLVWithHeader, error: %s\n", strerror(errno));
		}
		return status;
	}
};


#endif /* SOCKETHELPER_H_ */
