/*
 * =====================================================================================
 *
 *       Filename:  client.c
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  2013年08月19日 17时19分33秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <error.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>

#define SEVPORT 5000
#define MAXDATASIZE  (1024*5)

//#define CLIENT_DEBUG 1

char buf[MAXDATASIZE];

int main(int argc, char *argv[])
{
	int sockfd,sendbytes,recvbytes;
	struct hostent *host;
	struct sockaddr_in serv_addr;
	int loops = 0;
	if(argc < 2){
		fprintf(stderr,"Please enter the server's hostname!\n");
		exit(1);
	}

	if((host=gethostbyname(argv[1])) == NULL){
		perror("gethostbyname:");
		exit(1);
	}
#ifdef CLIENT_DEBUG
	printf("hostent h_name: %s , h_aliases: %s,\
			h_addrtype: %d, h_length: %d, h_addr_list: %s\n",\
			host->h_name,*(host->h_aliases),host->h_addrtype,host->h_length,*(host->h_addr_list));
#endif

	if((sockfd = socket(AF_INET,SOCK_STREAM,0)) == -1){
		perror("socket:");
		exit(1);
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(SEVPORT);
	serv_addr.sin_addr = *((struct in_addr *)host->h_addr);
	bzero(&(serv_addr.sin_zero),8);

	printf("Trying to connect...\n");
	if(connect(sockfd,(struct sockaddr *)&serv_addr, \
					sizeof(struct sockaddr)) == -1){
		perror("connect:");
		exit(1);
	}
	printf("Connected\n");
	while(1)
	{
        sprintf(buf,"B\n");
        if((sendbytes = send(sockfd,buf,strlen(buf),0)) == -1){
            perror("send:");
            exit(1);
        }
        bzero(buf,sizeof(buf));
        if((recvbytes = recv(sockfd, buf,MAXDATASIZE,0)) == -1)
        {
            perror("recv");
            close(sockfd);
            exit(1);
        }
        loops++;
        printf("%d : Client received bytes: %d -> %s",loops,recvbytes,buf);
        usleep(100000);
        sprintf(buf,"L\n");
        if((sendbytes = send(sockfd,buf,strlen(buf),0)) == -1){
            perror("send:");
            exit(1);
        }
        bzero(buf,sizeof(buf));
        if((recvbytes = recv(sockfd, buf,MAXDATASIZE,0)) == -1)
        {
            perror("recv");
            close(sockfd);
            exit(1);
        }
        loops++;
        printf("%d : Client received bytes: %d -> %s",loops,recvbytes,buf);
        usleep(100000);
	}
	close(sockfd);
}
