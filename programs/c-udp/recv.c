  

//for UDP
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <netinet/in.h>
#include <arpa/inet.h>
//#include <netdb.h>
#include <string.h>
#include <unistd.h>

#define BUFFER_SIZE 256



int main(int argc, char **argv)
{

	/* ポート番号、ソケット */
	unsigned short port = 4001;
	int recvSocket;


	/* sockaddr_in 構造体 */
	struct sockaddr_in recvSockAddr;


	/* 各種パラメータ */
	int status;
	int numrcv;
	char buffer[BUFFER_SIZE];
	unsigned long on = 1;


	/* sockaddr_in 構造体のセット */
	memset(&recvSockAddr, 0, sizeof(recvSockAddr));
	recvSockAddr.sin_port = htons(port);
	recvSockAddr.sin_family = AF_INET;
	//recvSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	recvSockAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

	/* ソケット生成 */
	recvSocket = socket(AF_INET, SOCK_DGRAM, 0);

	/* バインド */
	status = bind(recvSocket, (const struct sockaddr *) &recvSockAddr, sizeof(recvSockAddr));

	//select
	fd_set fds, readfds;
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	/* fd_setの初期化します */
	FD_ZERO(&readfds);
	/* selectで待つ読み込みソケットとしてsock1を登録します */
	FD_SET(recvSocket, &readfds);
	
	while (1){
		memcpy(&fds, &readfds, sizeof(fd_set));
		int n = select(recvSocket+1, &fds, NULL, NULL, &tv);
		if(n>0){
			if (FD_ISSET(recvSocket, &fds)) {
				memset(buffer, 0, BUFFER_SIZE);
				numrcv = recvfrom(recvSocket, buffer, BUFFER_SIZE, 0, NULL, NULL);
				if(numrcv == -1) { status = close(recvSocket); break; }
				printf("received: %s\n", buffer);
				
			}
		}
	} 
 	return 0;
}


