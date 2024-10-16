FROM i386/ubuntu:18.04

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# ENV DEBIAN_FRONTEND noninteractive
ENV LANG=en_US.utf8

RUN apt-get update
RUN apt-get install -y wget make gcc g++ gdb nano ssh git
#g++-arm-linux-gnueabihf

RUN apt-get install -y locales &&   \
    rm -rf /var/lib/apt/lists/* &&  \
    localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8 

RUN apt-get update && apt-get install -y    \
    iptables net-tools iproute2             \
    redis 

# RUN apt-get update && apt-get install -y hiredis 
RUN git clone http://github.com/redis/hiredis
RUN cd hiredis && \
    make &&\
    make install &&\
    ldconfig

WORKDIR /home/ethernet-handler

# Copy app to container
COPY ./ethernet-server-start.sh /etc/init.d/

# During debugging, this entry point will be overridden. For more information, please refer to https://aka.ms/vscode-docker-python-debug

# CMD ["bash"]
# CMD ["start.sh"]

# CMD service ssh restart && bash
# CMD service ethernet-server-startup.service restart        # && bash

CMD /etc/init.d/ethernet-server-start.sh restart

CMD /bin/bash
