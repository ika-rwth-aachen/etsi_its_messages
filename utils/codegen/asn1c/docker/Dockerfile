# docker build -t gitlab.ika.rwth-aachen.de:5050/fb-fi/definitions/etsi_its_messages/asn1c:latest .

FROM ubuntu:18.04

# install essentials
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        git \
        tar \
        wget \
    && rm -rf /var/lib/apt/lists/*

# install asn1c dependencies
RUN apt-get update && \
    apt-get install -y \
        automake-1.15 \
        flex \
        libtool \
        m4 \
    && rm -rf /var/lib/apt/lists/*

# install Bison 2.7
WORKDIR /setup
RUN wget http://ftp.gnu.org/gnu/bison/bison-2.7.tar.gz && \
    tar -xvzf bison-2.7.tar.gz
WORKDIR /setup/bison-2.7
RUN PATH=$PATH:/usr/local/m4/bin/ && \
    ./configure --prefix=/usr/local/bison --with-libiconv-prefix=/usr/local/libiconv/ && \
    make && \
    make install

# install asnc1c
WORKDIR /setup
RUN git clone https://github.com/vlm/asn1c.git
WORKDIR /setup/asn1c
ARG ASN1C_COMMIT=9925dbb
RUN git checkout ${ASN1C_COMMIT} && \
    test -f configure || autoreconf -iv && \
    ./configure && \
    make && \
    make install

# cleanup
WORKDIR /
RUN rm -rf /setup

# command
RUN mkdir input
RUN mkdir output
WORKDIR /output
RUN echo "asn1c \$(find /input -name '*.asn') -fcompound-names -no-gen-example -gen-PER" > /asn1c.sh
CMD ["/bin/bash", "/asn1c.sh"]
