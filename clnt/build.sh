#!/bin/bash

# compile static library
# gcc -Wall -c *.c && ar -cvq libdie_with_error.a die_with_error.o

# gcc -o client client.c -L. -ldie_with_error

gcc die_with_error.h die_with_error.c client.c -o client
