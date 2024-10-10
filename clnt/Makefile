CC=gcc
CFLAGS=-I.
DEPS=
OBJ=client.o
USERID=123456789

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: client
client: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -rf *.o client *.tar.gz

dist: tarball
tarball: clean
	tar -cvzf /tmp/$(USERID).tar.gz --exclude=./.vagrant . && mv /tmp/$(USERID).tar.gz .