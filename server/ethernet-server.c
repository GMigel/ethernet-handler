/*
 ============================================================================
 Name        : ethernet_server.c
 Author      : Mike
 Version     :
 Copyright   : copyright notice
 Description :
 ============================================================================
 */

#include <arpa/inet.h>
#include <hiredis/hiredis.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int redis_publish(char *buf, ssize_t length);

// C program to display hostname
// and IP address
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "in.h"
#include "packs.h"

// typedef struct udp_pack_broadcast_part udp_pack_broadcast_part;

// #include "/home/mike/projects/L29-MFD-12-GORIZONT-render/source/apps/l29/model/data/in.h"

// https://www.geeksforgeeks.org/c-program-display-hostname-ip-address/
// https://www.geeksforgeeks.org/tcp-server-client-implementation-in-c/
// https://github.com/JeffreytheCoder/Simple-HTTP-Server/tree/master
// https://www.codequoi.com/en/sockets-and-network-programming-in-c/

// int main_(int argc, char **argv);

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  //  char *tcp_server_ip = "127.0.0.1";
  //  char *tcp_server_ip = "172.17.0.12";

#define PORT1 17300
#define BUFSIZE 1500
  char hostbuffer[256];
  struct hostent *host_entry;
  int hostname, fd;
  unsigned int namelen;
  struct in_addr **addr_list;
  char *IPbuffer;
  struct sockaddr_in serveraddr; /** server's addr */
  struct sockaddr_in clientaddr; /** client addr */
  struct hostent *hostp;         /** client host info */
  char *hostaddrp;               /** dotted decimal host addr string */
  int clientlen;                 /** byte size of client's address */
  int num;                       /** message byte size */
  int sockfd;                    /** socket */
  int optval;                    /** flag value for setsockopt */
  int portno;                    /** port to listen on */
  //  int server_sock, client_sock;
  //  socklen_t addr_size;

  portno = PORT1;
  char buf[BUFSIZE];

  hostname = gethostname(hostbuffer, sizeof(hostbuffer));
  if (hostname == -1) {
    perror("gethostname error");
    exit(1);
  }

  host_entry = gethostbyname(hostbuffer);
  if (host_entry == NULL) {
    perror("getHostByName error");
    exit(1);
  }

  addr_list = (struct in_addr **)host_entry->h_addr_list;

  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));

  for (int i = 0; addr_list[i] != NULL; i++) {
    printf("IP address %d: %s\n", i + 1, inet_ntoa(*addr_list[i]));
  }
  printf("Host IP: %s\n", IPbuffer);

  //  server_sock = socket(AF_INET,
  //  SOCK_STREAM, 0);
  sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP); /** socket: create the parent socket */
  if (sockfd < 0) {
    perror("ERROR opening socket\n");
    exit(1);
  }

  /** setsockopt: Handy debugging trick that lets us rerun the server immediately after we kill it,
   * otherwise we have to wait about 20 secs. Eliminates "ERROR on binding: Address already in use" error.
   */
  optval = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(int));

  /** build the server's Internet address  */
  //  memset(&serveraddr, '\0', sizeof(serveraddr));
  bzero((char *)&serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  //  inet_aton(IPbuffer, (struct in_addr *)&serveraddr.sin_addr.s_addr); /** Get host's IP */
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY); /** Redefine IP to '0.0.0.0' */
  serveraddr.sin_port = htons((unsigned short)portno);

  //  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));

  /** bind: associate the parent socket with a port */
  if (bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
    perror("[-]Bind error");
    exit(2);
  }

  char stripv4[INET_ADDRSTRLEN];
  const char *result = inet_ntop(AF_INET, &serveraddr.sin_addr.s_addr, stripv4, INET_ADDRSTRLEN);
  if (result == NULL) {
    perror("inet_ntop");
    //    return 1;
  }
  printf("[+]Bind to the socket: %s:%d\n", stripv4, portno);

  /* Find out what port was really assigned and print it */
  namelen = sizeof(serveraddr);
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (getsockname(fd, (struct sockaddr *)&serveraddr, &namelen) < 0) {
    perror("getsockname()");
    exit(3);
  }
  //  printf("Port assigned is %d\n", ntohs(serveraddr.sin_port));

  /* Find out what port was really assigned and print it */
  //  namelen = sizeof(serveraddr);
  //  if (getsockname(s, (struct sockaddr *)&serveraddr, &namelen) < 0) {
  //    perror("getsockname()");
  //    exit(3);
  //  }
  //  printf("Port assigned is %d\n", ntohs(serveraddr.sin_port));

  clientlen = sizeof(clientaddr);

  while (1) {
    //    addr_size = sizeof(clientaddr);
    //    client_sock = accept(server_sock, (struct sockaddr *)&clientaddr, &addr_size);
    //    printf("[+]Client connected.\n");

    /** recvfrom: receive a UDP datagram from a client */
    bzero(buf, BUFSIZE);
    num = recvfrom(sockfd, buf, BUFSIZE, 0, (struct sockaddr *)&clientaddr, (socklen_t *)&clientlen);
    if (num < 0)
      perror("ERROR in recvfrom");

    /** gethostbyaddr: determine who sent the datagram */
    hostp = gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, sizeof(clientaddr.sin_addr.s_addr), AF_INET);
    if (hostp == NULL)
      perror("ERROR on gethostbyaddr");
    hostaddrp = inet_ntoa(clientaddr.sin_addr);
    if (hostaddrp == NULL)
      perror("ERROR on inet_ntoa\n");
    printf("server received datagram from %s (%s) %d bytes\n", hostp->h_name, hostaddrp, num);

    //    /** sendto: echo the input back to the client */
    //    n = sendto(sockfd, buf, strlen(buf), 0, (struct sockaddr *)&clientaddr, clientlen);
    //    if (n < 0) error("ERROR in sendto");

    //    if (len != -1) {
    //      // printf("Client: %s\t%d\n", buffer, len);
    //      for (int i = 0; i < len; i++)
    //        printf("%02x", buffer[i]);

    FILE *fptr;
    if ((fptr = fopen("buffer.bin", "wb")) == NULL) {
      printf("Error! opening file");
      exit(EXIT_FAILURE);
    } else {
      fwrite(buf, num, 1, fptr);
      fclose(fptr);
    }

    redis_publish(buf, num);
  }

  return EXIT_SUCCESS;
}

int redis_publish(char *buf, ssize_t length) {
  (void)buf;
  (void)length;
  redisContext *ctx;
  redisReply *reply;

  udp_pack_broadcast_part *udp_part1 = (udp_pack_broadcast_part *)buf;

  const char *hostname = "173.18.0.10";
  const int port = 6379;
  //  const char *password = "111";

  struct timeval timeout = {1, 500000}; // 1.5 seconds
  ctx = redisConnectWithTimeout(hostname, port, timeout);

  if (ctx == NULL || ctx->err) {
    if (ctx) {
      printf("Connection error: %s\n", ctx->errstr);
      redisFree(ctx);
    } else if (ctx->err) {
      printf("Connection error: can't allocate redis context\n");
    }
    exit(1);
  }

  /* AUTH */
  //  reply = redisCommand(ctx, "AUTH %s", password);
  //  printf("AUTH: %s\n", reply->str);
  //  freeReplyObject(reply);

  //  /* PING server */
  //  reply = redisCommand(ctx, "PING");
  //  printf("PING: %s\n", reply->str);
  //  freeReplyObject(reply);

  /* Set a key */
  reply = redisCommand(ctx, "SET %s %d", "pitch",
                       udp_part1->pitch); // pitch; 155 Угол тангажа: от -90 до 90 с шагом 4.23751771450042724609375E-08
  // и значением по-умолчанию 0
  printf("SET: 'pitch' %d, REPLY: %s\n", udp_part1->pitch, reply->str);
  freeReplyObject(reply);

  reply = redisCommand(
      ctx, "SET %s %d", "roll",
      udp_part1->roll); // 159 Угол крена:  от -180 до 180 с шагом 8.42846930027008E-08 и значением по-умолчанию 0
  printf("SET: 'roll' %d, REPLY: %s\n", udp_part1->roll, reply->str);
  freeReplyObject(reply);

  /* Try a GET  */
  reply = redisCommand(ctx, "GET name");
  printf("GET name: %s\n", reply->str);
  freeReplyObject(reply);

  /* Disconnects and frees the context
   */
  redisFree(ctx);

  return EXIT_SUCCESS;
}

// #include <arpa/inet.h>
// #include <hiredis/hiredis.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>
//
// int redis_publish(char *buf, ssize_t length);
//
//// C program to display hostname
//// and IP address
// #include <arpa/inet.h>
// #include <errno.h>
// #include <netdb.h>
// #include <netinet/in.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <unistd.h>
//
// #include "in.h"
//
//// #include "/home/mike/projects/L29-MFD-12-GORIZONT-render/source/apps/l29/model/data/in.h"
//
//// https://www.geeksforgeeks.org/c-program-display-hostname-ip-address/
//// https://www.geeksforgeeks.org/tcp-server-client-implementation-in-c/
//// https://github.com/JeffreytheCoder/Simple-HTTP-Server/tree/master/
//// https://www.codequoi.com/en/sockets-and-network-programming-in-c/
//
//// int main_(int argc, char **argv);
//
// int main(int argc, char **argv) {
//  (void)argc;
//  (void)argv;
//
//  //  char *tcp_server_ip = "127.0.0.1";
//  //  char *tcp_server_ip = "172.17.0.12";
//
// #define PORT1 17300
// #define BUFSIZE 1500
//  char hostbuffer[256];
//  struct hostent *host_entry;
//  int hostname, fd;
//  unsigned int namelen;
//  struct in_addr **addr_list;
//  char *IPbuffer;
//  struct sockaddr_in serveraddr; /** server's addr */
//  struct sockaddr_in clientaddr; /** client addr */
//  struct hostent *hostp;         /** client host info */
//  char *hostaddrp;               /** dotted decimal host addr string */
//  int clientlen;                 /** byte size of client's address */
//  int n;                         /** message byte size */
//  int sockfd;                    /** socket */
//  int optval;                    /** flag value for setsockopt */
//  int portno;                    /** port to listen on */
//  //  int server_sock, client_sock;
//  //  socklen_t addr_size;
//
//  portno = PORT1;
//  char buf[BUFSIZE];
//
//  hostname = gethostname(hostbuffer, sizeof(hostbuffer));
//  if (hostname == -1) {
//    perror("gethostname error");
//    exit(1);
//  }
//
//  host_entry = gethostbyname(hostbuffer);
//  if (host_entry == NULL) {
//    perror("getHostByName error");
//    exit(1);
//  }
//
//  addr_list = (struct in_addr **)host_entry->h_addr_list;
//
//  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));
//
//  for (int i = 0; addr_list[i] != NULL; i++) {
//    printf("IP address %d: %s\n", i + 1, inet_ntoa(*addr_list[i]));
//  }
//  printf("Host IP: %s\n", IPbuffer);
//
//  //  server_sock = socket(AF_INET,
//  //  SOCK_STREAM, 0);
//  sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP); /** socket: create the parent socket */
//  if (sockfd < 0) {
//    perror("ERROR opening socket\n");
//    exit(1);
//  }
//
//  /** setsockopt: Handy debugging trick that lets us rerun the server immediately after we kill it,
//   * otherwise we have to wait about 20 secs. Eliminates "ERROR on binding: Address already in use" error.
//   */
//  optval = 1;
//  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(int));
//
//  /** build the server's Internet address  */
//  //  memset(&serveraddr, '\0', sizeof(serveraddr));
//  bzero((char *)&serveraddr, sizeof(serveraddr));
//  serveraddr.sin_family = AF_INET;
//  inet_aton(IPbuffer, (struct in_addr *)&serveraddr.sin_addr.s_addr); /** Get host's IP */
//  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);                     /** Redefine IP to '0.0.0.0' */
//  serveraddr.sin_port = htons((unsigned short)portno);
//
//  //  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));
//
//  /** bind: associate the parent socket with a port */
//  if (bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
//    perror("[-]Bind error");
//    exit(2);
//  }
//
//  char stripv4[INET_ADDRSTRLEN];
//  const char *result = inet_ntop(AF_INET, &serveraddr.sin_addr.s_addr, stripv4, INET_ADDRSTRLEN);
//  if (result == NULL) {
//    perror("inet_ntop");
//    //    return 1;
//  }
//  printf("[+]Bind to the socket: %s:%d\n", stripv4, portno);
//
//  /* Find out what port was really assigned and print it */
//  namelen = sizeof(serveraddr);
//  fd = socket(AF_INET, SOCK_STREAM, 0);
//  if (getsockname(fd, (struct sockaddr *)&serveraddr, &namelen) < 0) {
//    perror("getsockname()");
//    exit(3);
//  }
//  printf("Port assigned is %d\n", ntohs(serveraddr.sin_port));
//
//  /* Find out what port was really assigned and print it */
//  //  namelen = sizeof(serveraddr);
//  //  if (getsockname(s, (struct sockaddr *)&serveraddr, &namelen) < 0) {
//  //    perror("getsockname()");
//  //    exit(3);
//  //  }
//  //  printf("Port assigned is %d\n", ntohs(serveraddr.sin_port));
//
//  clientlen = sizeof(clientaddr);
//
//  while (1) {
//    //    addr_size = sizeof(clientaddr);
//    //    client_sock = accept(server_sock, (struct sockaddr *)&clientaddr, &addr_size);
//    //    printf("[+]Client connected.\n");
//
//    /** recvfrom: receive a UDP datagram from a client */
//    bzero(buf, BUFSIZE);
//    n = recvfrom(sockfd, buf, BUFSIZE, 0, (struct sockaddr *)&clientaddr, (socklen_t *)&clientlen);
//    if (n < 0)
//      perror("ERROR in recvfrom");
//
//    /** gethostbyaddr: determine who sent the datagram */
//    hostp = gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, sizeof(clientaddr.sin_addr.s_addr), AF_INET);
//    if (hostp == NULL)
//      perror("ERROR on gethostbyaddr");
//    hostaddrp = inet_ntoa(clientaddr.sin_addr);
//    if (hostaddrp == NULL)
//      perror("ERROR on inet_ntoa\n");
//    printf("server received datagram from %s (%s)\n", hostp->h_name, hostaddrp);
//    printf("server received %d/%d bytes: %s\n", (int)strlen(buf), n, buf);
//
//    //    /** sendto: echo the input back to the client */
//    //    n = sendto(sockfd, buf, strlen(buf), 0, (struct sockaddr *)&clientaddr, clientlen);
//    //    if (n < 0) error("ERROR in sendto");
//
//    //    if (len != -1) {
//    //      // printf("Client: %s\t%d\n", buffer, len);
//    //      for (int i = 0; i < len; i++)
//    //        printf("%02x", buffer[i]);
//
//    FILE *fptr;
//    if ((fptr = fopen("buffer.bin", "wb")) == NULL) {
//      printf("Error! opening file");
//      exit(EXIT_FAILURE);
//    } else {
//      fwrite(buf, n, 1, fptr);
//      fclose(fptr);
//    }
//  }
//
//  //  bzero(buffer, 1024);
//  //  //    strcpy(buffer, "HI, THIS IS
//  //  //    SERVER. HAVE A NICE DAY!!!");
//  //  printf("Server: %s\n", buffer);
//  //  send(client_sock, buffer, strlen(buffer), 0);
//  //  close(client_sock);
//  //  printf("[+]Client disconnected.\n\n");
//  redis_publish(buf, n);
//
//  return EXIT_SUCCESS;
//}
//
// int redis_publish(char *buf, ssize_t length) {
//  (void)buf;
//  (void)length;
//
//  redisContext *ctx;
//  redisReply *reply;
//
//  const char *hostname = "173.18.0.10"; // argv[1];
//  const int port = 6379;                // atoi(argv[2]);
//  const char *password = "111";         // argv[4];
//
//  struct timeval timeout = {1, 500000}; // 1.5 seconds
//  ctx = redisConnectWithTimeout(hostname, port, timeout);
//
//  // if (ctx == NULL || ctx->err) {
//  if (ctx) {
//    printf("Connection error: %s\n", ctx->errstr);
//    redisFree(ctx);
//  } else if (ctx->err) {
//    printf("Connection error: can't "
//           "allocate redis context\n");
//  }
//  exit(1);
//  //}
//
//  /* AUTH */
//  reply = redisCommand(ctx, "AUTH %s", password);
//  printf("AUTH: %s\n", reply->str);
//  freeReplyObject(reply);
//
//  /* PING server */
//  reply = redisCommand(ctx, "PING");
//  printf("PING: %s\n", reply->str);
//  freeReplyObject(reply);
//
//  /* Set a key */
//  reply = redisCommand(ctx, "SET %s %s", "name", "credis_test");
//  printf("SET: %s\n", reply->str);
//  freeReplyObject(reply);
//
//  /* Try a GET  */
//  reply = redisCommand(ctx, "GET name");
//  printf("GET name: %s\n", reply->str);
//  freeReplyObject(reply);
//
//  /* Disconnects and frees the context
//   */
//  redisFree(ctx);
//
//  return EXIT_SUCCESS;
//}
