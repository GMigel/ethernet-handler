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

//// Returns hostname for the local computer
// void checkHostName(int hostname) {
//   if (hostname == -1) {
//     perror("gethostname");
//     exit(1);
//   }
// }
//
//// Returns host information corresponding to host name
// void checkHostEntry(struct hostent *hostentry) {
//   if (hostentry == NULL) {
//     perror("gethostbyname");
//     exit(1);
//   }
// }
//
//// Converts space-delimited IPv4 addresses
//// to dotted-decimal format
// void checkIPbuffer(char *IPbuffer) {
//   if (NULL == IPbuffer) {
//     perror("inet_ntoa");
//     exit(1);
//   }
// }

// Driver code
int main() {

  char hostbuffer[256];
  struct hostent *host_entry;
  int hostname;
  struct in_addr **addr_list;
  char *IPbuffer;

  // retrieve hostname
  hostname = gethostname(hostbuffer, sizeof(hostbuffer));
  if (hostname == -1) {
    perror("gethostname error");
    exit(1);
  }
  //  printf("Hostname: %s\n", hostbuffer);

  // Retrieve IP addresses
  host_entry = gethostbyname(hostbuffer);
  if (host_entry == NULL) {
    perror("gethostbyname error");
    exit(1);
  }

  addr_list = (struct in_addr **)host_entry->h_addr_list;

  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));

  for (int i = 0; addr_list[i] != NULL; i++) {
    printf("IP address %d: %s\n", i + 1, inet_ntoa(*addr_list[i]));
  }
  printf("Host IP: %s\n", IPbuffer);

  //  char hostbuffer[256];
  //  char *IPbuffer;
  //  struct hostent *host_entry;
  //  int hostname;
  //
  //  // To retrieve hostname
  //  hostname = gethostname(hostbuffer, sizeof(hostbuffer));
  //  checkHostName(hostname);
  //
  //  // To retrieve host information
  //  host_entry = gethostbyname(hostbuffer);
  //  checkHostEntry(host_entry);
  //
  //  // To convert an Internet network
  //  // address into ASCII string
  //  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));
  //
  //  printf("Hostname: %s\n", hostbuffer);
  //  printf("Host IP: %s", IPbuffer);

  //

  printf("\n");

  //  char *tcp_server_ip = "127.0.0.1";
  //  char *tcp_server_ip = "172.17.0.12";
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

  IPbuffer = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0]));

  n = bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (n < 0) {
    perror("[-]Bind error");
    exit(1);
  }
  printf("[+]Bind to the port number: %d\t%d\n",
         (uint32_t)server_addr.sin_addr.s_addr, port);

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
  (void)ctx;
  (void)reply;
  (void)redis_hostname;
  (void)buf;
  (void)length;

  return EXIT_SUCCESS;
}
