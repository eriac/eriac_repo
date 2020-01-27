//#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <unistd.h>

#include <string.h>


int sock;
struct sockaddr_in addr;
void udp_init(char *address, int port){
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(address);
	addr.sin_port = htons(port);
}

void udp_send(char *data){
	sendto(sock, data, strlen(data), 0, (struct sockaddr *)&addr, sizeof(addr));
}

void udp_bind(){
	bind(sock, (struct sockaddr *)&addr, sizeof(addr));
}
void udp_recv(char *buf, int size){
	memset(buf, 0, size);
	recv(sock, buf, size, 0);
}

void udp_clone(){
	close(sock);
}
/*
int main(){
	udp_init("127.0.0.1",4001);
	udp_send("hello world!");
	udp_clone();

	return 0;
}
*/
int main(){
	udp_init("0.0.0.0",4001);
	udp_bind();
	for(int i=0;i<4;i++){
		char buf[2048];
		udp_recv(buf,2048);
		printf("%s\n",buf);
	}
	udp_clone();

	return 0;
}
