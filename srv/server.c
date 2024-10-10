/*
 ============================================================================
 Name        : server.c
 Author      : Mike
 Version     :
 Copyright   :
 Description : https://blog.codefarm.me/2021/10/20/sockets-programming-in-c-using-tcp-ip/
 ============================================================================
 */

/* tcp_server.c */
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "die_with_error.h"

#define PORT 15100
#define MAXPENDING 1024
#define RCVBUFSIZE 4096

int main(void) {
  /* Create a TCP socket */
  /* Create socket for incoming connections */
  int serv_sock;
  if ((serv_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    die_with_error("socket() failed");
  }

  /* Assign a port to socket */
  int serv_port = PORT;
  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;                /* Internet address family */
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
  serv_addr.sin_port = htons(serv_port);         /* Local port */
  if (bind(serv_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    die_with_error("bind() failed");
  }

  /* Set socket to listen */
  /* Mark the socket so it will listen for incoming connections */
  if (listen(serv_sock, MAXPENDING) < 0) {
    die_with_error("listen() failed");
  }

  /* Repeatedly: */
  /* Run forever */
  for (;;) {
    /* Accept new connectionb */
    /* Server is now blocked waiting for connection from a client */
    int client_sock;
    struct sockaddr client_addr;
    int addr_len;
    addr_len = sizeof(client_addr);
    if ((client_sock = accept(serv_sock, &client_addr, (socklen_t *)&addr_len)) < 0) {
      die_with_error("accept() failed");
    }

    struct sockaddr_in *c_addr = (struct sockaddr_in *)&client_addr;
    char c_ip_addr[INET6_ADDRSTRLEN];
    inet_ntop(c_addr->sin_family, &(c_addr->sin_addr), c_ip_addr, addr_len);
    int c_port = ntohs(c_addr->sin_port);
    printf("%s:%d Connected\n", c_ip_addr, c_port);

    /* Receive mesage from client */
    int recv_msg_size;
    char echo_buf[RCVBUFSIZE];
    if ((recv_msg_size = recv(client_sock, echo_buf, RCVBUFSIZE, 0)) < 0) {
      die_with_error("first recv() failed");
    }

    /* Send received string and receive again until end of transmission */
    while (recv_msg_size > 0) { /* zero indicates end of transmission */
      if (send(client_sock, echo_buf, recv_msg_size, 0) != recv_msg_size) {
        die_with_error("repeat send() failed");
      }
      printf("%s\n", echo_buf);
      memset(echo_buf, '\0', RCVBUFSIZE);

      if ((recv_msg_size = recv(client_sock, echo_buf, RCVBUFSIZE, 0)) < 0) {
        die_with_error("recv() failed");
      }
    }

    /* Close the connection */
    close(client_sock);
    printf("%s:%d Disconnected\n", c_ip_addr, c_port);
  }
}

// #include <netdb.h>
// #include <netinet/in.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <unistd.h> // read(), write(), close()
// #define MAX 1024
// #define PORT 17000
// #define SA struct sockaddr
//
//// Function designed for chat between client and server.
// void func(int connfd) {
//   char buff[MAX];
//   //  int n;
//   // infinite loop for chat
//   for (;;) {
//     bzero(buff, MAX);
//
//     // read the message from client and copy it in buffer
//     // read(connfd, buff, sizeof(buff));
//     char *answer = "Server's answer.";
//     memcpy(buff, answer, strlen(answer));
//     // print buffer which contains the client contents
//     printf("From server: %s\t To client : ", buff);
//     bzero(buff, MAX);
//
//     // and send that buffer to client
//     write(connfd, buff, sizeof(buff));
//
//     // if msg contains "Exit" then server exit and chat ended.
//     if (strncmp("exit", buff, 4) == 0) {
//       printf("Server Exit...\n");
//       break;
//     }
//   }
// }
//
//// Driver function
// int main() {
//   int sockfd, connfd, len;
//   struct sockaddr_in servaddr, cli;
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
//   servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
//   servaddr.sin_port = htons(PORT);
//
//   // Binding newly created socket to given IP and verification
//   if ((bind(sockfd, (SA *)&servaddr, sizeof(servaddr))) != 0) {
//     printf("socket bind failed...\n");
//     exit(0);
//   } else
//     printf("Socket successfully binded..\n");
//
//   // Now server is ready to listen and verification
//   if ((listen(sockfd, 5)) != 0) {
//     printf("Listen failed...\n");
//     exit(0);
//   } else
//     printf("Server listening..\n");
//   len = sizeof(cli);
//
//   // Accept the data packet from client and verification
//   connfd = accept(sockfd, (SA *)&cli, (socklen_t *)&len);
//   if (connfd < 0) {
//     printf("server accept failed...\n");
//     exit(0);
//   } else
//     printf("server accept the client...\n");
//
//   // Function for chatting between client and server
//   func(connfd);
//
//   // After chatting close the socket
//   close(sockfd);
// }
