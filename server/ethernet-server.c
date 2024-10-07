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

int 

redis_publish(char *buf, ssize_t length);

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

// #include
// "/home/mike/projects/L29-MFD-12-GORIZONT-render/source/apps/l29/model/data/in.h"

// https://www.geeksforgeeks.org/c-program-display-hostname-ip-address/
// https://www.geeksforgeeks.org/tcp-server-client-implementation-in-c/
// https://github.com/JeffreytheCoder/Simple-HTTP-Server/tree/master
// https://www.codequoi.com/en/sockets-and-network-programming-in-c/

// int main_(int argc, char **argv);

int main(int argc, char **argv) {

  //  int err = main_(argc, argv);
  //  (void)err;
  //  return (EXIT_SUCCESS);

  char hostbuffer[256];
  struct hostent *host_entry;
  int hostname;
  struct in_addr **addr_list;
  char *IPbuffer;

  hostname = gethostname(
      hostbuffer, sizeof(hostbuffer));
  if (hostname == -1) {
    perror("gethostname error");
    exit(1);
  }

  host_entry =
      gethostbyname(hostbuffer);
  if (host_entry == NULL) {
    perror("gethostbyname error");
    exit(1);
  }

  addr_list =
      (struct in_addr **)
          host_entry->h_addr_list;

  IPbuffer = inet_ntoa(*(
      (struct in_addr *)
          host_entry->h_addr_list[0]));

  for (int i = 0; addr_list[i] != NULL;
       i++) {
    printf("IP address %d: %s\n", i + 1,
           inet_ntoa(*addr_list[i]));
  }
  printf("Host IP: %s\n", IPbuffer);

  //  char *tcp_server_ip = "127.0.0.1";
  //  char *tcp_server_ip =
  //  "172.17.0.12";
  int port = 17300;

  int server_sock, client_sock;
  struct sockaddr_in server_addr,
      client_addr;
  socklen_t addr_size;
  char buffer[1024];
  int n;
  ssize_t len;

  //  server_sock = socket(AF_INET,
  //  SOCK_STREAM, 0);
  server_sock =
      socket(PF_INET, SOCK_STREAM,
             IPPROTO_TCP);

  if (server_sock < 0) {
    perror("[-]Socket error");
    exit(1);
  }
  //  printf("[+]TCP server socket
  //  created.\n");

  memset(&server_addr, '\0',
         sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr =
      htonl(INADDR_ANY);
  inet_aton(
      IPbuffer,
      (struct in_addr *)&server_addr
          .sin_addr.s_addr);
  //  server_addr.sin_port = htons(179);
  server_addr.sin_port = port;

  //  IPbuffer = inet_ntoa(*((struct
  //  in_addr
  //  *)host_entry->h_addr_list[0]));

  n = bind(
      server_sock,
      (struct sockaddr *)&server_addr,
      sizeof(server_addr));
  if (n < 0) {
    perror("[-]Bind error");
    exit(1);
  }

  char stripv4[INET_ADDRSTRLEN];
  const char *result = inet_ntop(
      AF_INET,
      &server_addr.sin_addr.s_addr,
      stripv4, INET_ADDRSTRLEN);
  if (result == NULL) {
    perror("inet_ntop");
    //    return 1;
  }
  printf("[+]Bind to the port number: "
         "%s\t%d\n",
         stripv4, port);

  listen(server_sock, 5);
  printf("Listening ...\n");

  while (1) {
    addr_size = sizeof(client_addr);
    client_sock = accept(
        server_sock,
        (struct sockaddr *)&client_addr,
        &addr_size);
    printf("[+]Client connected.\n");

    bzero(buffer, 1024);
    len = recv(client_sock, buffer,
               sizeof(buffer), 0);
    printf("Client: %s\t%ld\n", buffer,
           len);

    bzero(buffer, 1024);
    //    strcpy(buffer, "HI, THIS IS
    //    SERVER. HAVE A NICE DAY!!!");
    printf("Server: %s\n", buffer);
    send(client_sock, buffer,
         strlen(buffer), 0);
    redis_publish(buffer, len);

    close(client_sock);
    printf(
        "[+]Client disconnected.\n\n");
  }

  return EXIT_SUCCESS;
}

int redis_publish(char *buf,
                  ssize_t length) { /*{
redisContext *ctx;
redisReply *reply;
const char *redis_hostname =
"127.0.0.1"; (void)ctx; (void)reply;
(void)redis_hostname;
(void)buf;
(void)length;

return EXIT_SUCCESS;
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hiredis/hiredis.h>

// int main(int argctx, char **argv) {
//
https://www.tencentcloud.com/document/product/239/7046
int main_(int argc, char **argv) {
if (argc < 4) {
printf("Usage: 192.xx.xx.195 6379
instance_id password\n"); exit(0);
}
*/

  redisContext *ctx;
  redisReply *reply;

  const char *hostname =
      "173.18.0.10"; // argv[1];
  const int port =
      6379; // atoi(argv[2]);
  const char *password =
      "111"; // argv[4];

  struct timeval timeout = {
      1, 500000}; // 1.5 seconds
  ctx = redisConnectWithTimeout(
      hostname, port, timeout);

  // if (ctx == NULL || ctx->err) {
  if (ctx) {
    printf("Connection error: %s\n",
           ctx->errstr);
    redisFree(ctx);
  } else if (ctx->err) {
    printf("Connection error: can't "
           "allocate redis context\n");
  }
  exit(1);
  //}

  /* AUTH */
  reply = redisCommand(ctx, "AUTH %s",
                       password);
  printf("AUTH: %s\n", reply->str);
  freeReplyObject(reply);

  /* PING server */
  reply = redisCommand(ctx, "PING");
  printf("PING: %s\n", reply->str);
  freeReplyObject(reply);

  /* Set a key */
  reply = redisCommand(ctx, "SET %s %s",
                       "name",
                       "credis_test");
  printf("SET: %s\n", reply->str);
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
