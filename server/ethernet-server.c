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

int main() {

  char *tcp_server_ip = "127.0.0.1";
  //  char *tcp_server_ip = "172.17.0.2";
  int port = 17300;

  int server_sock, client_sock;
  struct sockaddr_in server_addr, client_addr;
  socklen_t addr_size;
  char buffer[1024];
  int n;
  ssize_t len;

  //  server_sock = socket(AF_INET, SOCK_STREAM, 0);
  server_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (server_sock < 0) {
    perror("[-]Socket error");
    exit(1);
  }
  printf("[+]TCP server socket created.\n");

  memset(&server_addr, '\0', sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  //  server_addr.sin_addr.s_addr = inet_addr(tcp_server_ip);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  //  server_addr.sin_port = htons(179);
  server_addr.sin_port = port;

  n = bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (n < 0) {
    perror("[-]Bind error");
    exit(1);
  }
  printf("[+]Bind to the port number: %s\t%d\n", tcp_server_ip, port);

  listen(server_sock, 5);
  printf("Listening...\n");

  while (1) {
    addr_size = sizeof(client_addr);
    client_sock =
        accept(server_sock, (struct sockaddr *)&client_addr, &addr_size);
    printf("[+]Client connected.\n");

    bzero(buffer, 1024);
    len = recv(client_sock, buffer, sizeof(buffer), 0);
    printf("Client: %s\n", buffer);

    bzero(buffer, 1024);
    strcpy(buffer, "HI, THIS IS SERVER. HAVE A NICE DAY!!!");
    printf("Server: %s\n", buffer);
    send(client_sock, buffer, strlen(buffer), 0);
    redis_publish(buffer, len);

    close(client_sock);
    printf("[+]Client disconnected.\n\n");
  }

  return EXIT_SUCCESS;
}

int redis_publish(char *buf, ssize_t length) {
  redisContext *ctx;
  redisReply *reply;
  const char *redis_hostname = "127.0.0.1";
  (void) ctx;
  (void) reply;
  (void) redis_hostname;
  (void) buf;
  (void) length;

  return EXIT_SUCCESS;
}
