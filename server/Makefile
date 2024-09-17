TARGET   =ethernet-server
CC       =gcc
CPPFLAGS =-I.
CFLAGS 	 =-W -Wall
# -ansi -pedantic
LDFLAGS  =-lhiredis
SOURCES  =$(wildcard *.c)

$(TARGET):
	$(CC) $(SOURCES) -o $@ $< $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o

# # BUILD_DIR = build
# # SOURCES = ...
# # TARGET  = ...
# # OBJECTS = $(SOURCES:%.c=$(BUILD_DIR)/%.o)

# # default: target

# # target: $(TARGET)

# # $(TARGET): $(OBJECTS)
# #     $(LD) -o $@ $(LDFLAGS) $^ $(LIBS)

# # $(BUILD_DIR)/%.o: %.c
# #     $(CC) -c -o $@ $< $(CFLAGS)

# # $(BUILD_DIR):
# #     -mkdir $@

# TARGET  = subscriber
# CC       =gcc
# BUILD_DIR = build
# CPPFLAGS =-I.
# CFLAGS   =-W -Wall -ansi -pedantic
# LDFLAGS  =-I -lhiredis
# # /usr/local/include/hiredis
# SOURCES  =$(wildcard *.c)
# EXECS    =$(SOURCES:%.c=%)

# # .PHONY: clean
# # clean:
# # 	$(RM) *~ *.o client

# # OBJECTS=TARGET.o

# # $(TARGET): $(OBJECTS)
# # 	$(CC) $(CFLAGS) -o TARGET $(LDFLAGS)

# $(TARGET):%.o: %.c
# # $(BUILD_DIR)/%.o: %.c
# 	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# # $(OBJECTS)

# all: $(TARGET)


# # TARGET  = subscriber
# # CC      = gcc
# # LD      = gcc
# # # CPPFLAGS = -I
# # # CFLAGS  = -W -Wall -ansi -pedantic
# # CFLAGS  =
# # LDFLAGS = -I /usr/local/include/hiredis -lhiredis
# # SOURCES = $(wildcard *.c)
# # LIBS	= 

# # EXECS   = $(SOURCES:%.c=%)

# # # all: $(EXECS)

# # default: target

# # target: $(TARGET)


# # default: target

# # target: $(TARGET)

# # # $(TARGET): $(OBJECTS)
# # #     $(LD) -o $@ $(LDFLAGS) $^ $(LIBS)

# # $(BUILD_DIR)/%.o: %.c
# #     $(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# # $(BUILD_DIR):
# #     -mkdir $@




# # BUILD_DIR = build
# # SOURCES = ...
# # TARGET  = ...
# # OBJECTS = $(SOURCES:%.c=$(BUILD_DIR)/%.o)

# # default: target

# # target: $(TARGET)

# # $(TARGET): $(OBJECTS)
# #     $(LD) -o $@ $(LDFLAGS) $^ $(LIBS)

# # $(BUILD_DIR)/%.o: %.c
# #     $(CC) -c -o $@ $< $(CFLAGS)

# # $(BUILD_DIR):
# #     -mkdir $@
