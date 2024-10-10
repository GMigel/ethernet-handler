/*
 ============================================================================
 Name        : client.c
 Author      : Mike
 Version     :
 Copyright   :
 Description : https://blog.codefarm.me/2021/10/20/sockets-programming-in-c-using-tcp-ip/
 ============================================================================
 */

/* tcp_client.c */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "die_with_error.h"

#define PORT 15100
#define SERVER_IP "127.0.0.1"

#define RCVBUFSIZE 4096

int main(void) {
  /* Create a TCP socket */
  /* Create a reliable, stream socket using TCP */
  int client_sock;
  if ((client_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    die_with_error("socket() failed");
  }

  /* Establish connection */
  char *serv_ip = SERVER_IP;
  int serv_port = PORT;
  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;                 /* Internet address family */
  serv_addr.sin_addr.s_addr = inet_addr(serv_ip); /* Server IP address*/
  serv_addr.sin_port = htons(serv_port);          /* Server port */
  if (connect(client_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    die_with_error("connect() failed");
  }

  /* Communicate */
  int read_len;
  char *read_buf = (char *)malloc(RCVBUFSIZE * sizeof(char));
  int recv_msg_size;
  char echo_buf[RCVBUFSIZE];
  for (;;) {
    read_buf = fgets(read_buf, sizeof(read_buf), stdin);
    if (read_buf == NULL) {
      exit(EXIT_SUCCESS);
    }

    read_len = strlen(read_buf); /* Determine input length */ /* Send the string to the server */
    if (send(client_sock, read_buf, read_len, 0) != read_len) {
      die_with_error("send() sent a different number of bytes than expected");
    }

    /* Receive mesage from server */
    if ((recv_msg_size = recv(client_sock, echo_buf, RCVBUFSIZE, 0)) < 0) {
      die_with_error("recv() failed");
    } else {
      printf("Server's answer: ");
      fputs(echo_buf, stdout);
    }
    memset(read_buf, '\0', RCVBUFSIZE);
    memset(echo_buf, '\0', RCVBUFSIZE);
  }

  /* Close the connection */
  close(client_sock);
}

// #include <arpa/inet.h> // inet_addr()
// #include <netdb.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <strings.h> // bzero()
// #include <sys/socket.h>
// #include <unistd.h> // read(), write(), close()
//
// #define MAX 1024
// #define PORT 17000
// #define SA struct sockaddr
//
// void func(int sockfd) {
//   char buff[MAX];
//   //  for (;;)
//   {
//     bzero(buff, sizeof(buff));
//     // printf("Enter the string : "); n = 0; while ((buff[n++] = getchar()) != '\n');
//     char *request = "Client's request.";
//
//     memcpy(buff, request, strlen(request));
//     write(sockfd, buff, sizeof(buff));
//     printf("From Client: %s", buff);
//
//     bzero(buff, sizeof(buff));
//     read(sockfd, buff, sizeof(buff));
//     printf("From Server : %s", buff);
//     //    if ((strncmp(buff, "exit", 4)) == 0) {
//     printf("Client Exit...\n");
//     //      break;
//     //    }
//   }
// }
//
// int main() {
//   int sockfd;                  //, connfd;
//   struct sockaddr_in servaddr; // cli;
//
//   // socket create and verification
//   sockfd = socket(AF_INET, SOCK_STREAM, 0);
//   if (sockfd == -1) {
//     printf("socket creation failed...\n");
//     exit(0);
//   } else
//     printf("Socket successfully created..\n");
//   bzero(&servaddr, sizeof(servaddr));
//
//   // assign IP, PORT
//   servaddr.sin_family = AF_INET;
//   servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
//   servaddr.sin_port = htons(PORT);
//
//   // connect the client socket to server socket
//   if (connect(sockfd, (SA *)&servaddr, sizeof(servaddr)) != 0) {
//     printf("connection with the server failed...\n");
//     exit(0);
//   } else
//     printf("connected to the server..\n");
//
//   // function for chat
//   func(sockfd);
//
//   // close the socket
//   close(sockfd);
// }
