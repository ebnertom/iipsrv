FROM ubuntu:16.04

LABEL MAINTAINER="KML VISION, devops@kmlvision.com"

ENV LANG C.UTF-8
ENV DEBIAN_FRONTEND noninteractive

# install build deps
RUN sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list && \
    apt-get update -qq && \
    apt-get -qq -y install \
      build-essential \
      autoconf \
      autoconf-archive \
      automake \
      openssl \
      locate \
      libssl-dev \
      net-tools \
      nano \
      cmake \
      git \
      libjpeg8-dev \
      libmemcached-dev \
      libopenjpeg-dev \
      libssl-dev \
      libtiff5-dev \
      pkg-config \
      psmisc \
      software-properties-common \
      nginx && \
    apt-get -y build-dep iipimage

WORKDIR /usr/src/iipsrv
RUN git clone https://github.com/kmlvision/iipsrv.git . && git checkout tags/iipsrv-1.0 && sh autogen.sh && ./configure && make
RUN cp ./src/iipsrv.fcgi /usr/local/bin/iipsrv.fcgi
RUN ldconfig -v

# set nginx config
COPY nginx.conf /etc/nginx/nginx.conf
# copy entrypoint
COPY start_iipsrv.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /
EXPOSE 80
ENTRYPOINT ["/entrypoint.sh"]