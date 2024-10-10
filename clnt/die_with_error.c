/* die_with_error.c */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void die_with_error(char *error) {
  int errsv = errno;
  fprintf(stderr, "%s: %s", strerror(errsv), error);
  exit(EXIT_FAILURE);
}
