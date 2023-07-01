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
#include <termios.h>

#define SERVER_PORT     5000
#define MAXDATASIZE     512

char tx_buf[4];
char rx_buf[MAXDATASIZE];
FILE    *csv_fp;

int kbhit(void)
{
struct termios oldt, newt;
int ch;
int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int net_connect(char *server_addr)
{
struct hostent *host;
int sockfd;
struct sockaddr_in serv_addr;

	if((host=gethostbyname(server_addr)) == NULL){
		perror("gethostbyname:");
		exit(1);
	}
	if((sockfd = socket(AF_INET,SOCK_STREAM,0)) == -1){
		perror("socket:");
		exit(1);
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(SERVER_PORT);
	serv_addr.sin_addr = *((struct in_addr *)host->h_addr);
	bzero(&(serv_addr.sin_zero),8);

	printf("Trying to connect...\n");
	if(connect(sockfd,(struct sockaddr *)&serv_addr, \
					sizeof(struct sockaddr)) == -1){
		perror("connect:");
		exit(1);
	}
	printf("Connected on %s : %d\n",server_addr,SERVER_PORT);
	return sockfd;
}

int get_sensors(char sensor,int sockfd)
{
int sendbytes,recvbytes;
    sprintf(tx_buf,"%c",sensor);
    if((sendbytes = send(sockfd,tx_buf,strlen(tx_buf),0)) == -1){
        perror("send:");
        return(1);
    }
    bzero(rx_buf,sizeof(rx_buf));
    if((recvbytes = recv(sockfd, rx_buf,MAXDATASIZE,0)) == -1)
    {
        perror("recv");
        return(1);
    }
    return 0;
}

int close_socket_and_file(int sockfd,int reason)
{
    if ( reason == 0 )
        printf("\nStopped from user\n");
    else
        printf("\nStopped for error\n");
    close(sockfd);
    fclose(csv_fp);
    exit (reason);
}

void store_data(int loops)
{
    fprintf(csv_fp,"%s",rx_buf);
    printf("%d : Client received %s",loops,rx_buf);
    usleep(100000);
}

int main(int argc, char *argv[])
{
int sockfd;
int loops = 0;
char    file_name[128],c;

	if(argc < 2)
	{
		fprintf(stderr,"Please enter the server's hostname!\n");
		exit(1);
	}
    sockfd = net_connect(argv[1]);

	sprintf(file_name,"sensors.csv");
	csv_fp = fopen(file_name,"w");
	if ( csv_fp == NULL )
	{
        printf("Error opening %s\n",file_name);
        exit(-1);
	}

	while(1)
	{
        if ( get_sensors('B',sockfd) != 0 )
            close_socket_and_file(sockfd,1);
        store_data(loops);
        if ( get_sensors('L',sockfd) != 0 )
            close_socket_and_file(sockfd,1);
        store_data(loops);
        loops++;
        if(kbhit())
        {
            c = getchar();
            if ( c == 's' )
                close_socket_and_file(sockfd,0);
        }
	}
}
