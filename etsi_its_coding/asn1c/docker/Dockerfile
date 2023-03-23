FROM ubuntu:18.04

ARG ASN1C_COMMIT

USER root

# Install git
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    git \
    wget \
    tar && \
    rm -rf /var/lib/apt/lists/*

# Install ASN1C dependencies
RUN apt-get update && \
    apt-get install -y \
    automake-1.15 \
    libtool \
    m4 \
    flex && \
    rm -rf /var/lib/apt/lists/*

# Install Bison 2.7
WORKDIR /setup
RUN wget http://ftp.gnu.org/gnu/bison/bison-2.7.tar.gz
RUN tar -xvzf bison-2.7.tar.gz
WORKDIR /setup/bison-2.7
RUN PATH=$PATH:/usr/local/m4/bin/
RUN ./configure --prefix=/usr/local/bison --with-libiconv-prefix=/usr/local/libiconv/
RUN make && make install

# Install asnc1c
WORKDIR /setup
RUN git clone https://github.com/vlm/asn1c.git
WORKDIR /setup/asn1c
RUN if [ -z ${ASN1C_COMMIT+x} ]; then echo "No specific commit for asn1c defined, utilizing latest commit on default branch!"; \
    else git checkout ${ASN1C_COMMIT}; fi
RUN test -f configure || autoreconf -iv
RUN ./configure
RUN make
RUN make install

# Clean-Up
WORKDIR /
RUN rm -rf /setup

# Copy Entrypoint
COPY entrypoint.sh /usr/bin/

WORKDIR /home
RUN mkdir input
RUN mkdir output

ENTRYPOINT ["/usr/bin/entrypoint.sh"]
CMD [""]


