# docker build -t ghcr.io/ika-rwth-aachen/etsi_its_messages:asn1c -f asn1c.Dockerfile .

FROM ubuntu:22.04

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
        automake \
        bison \
        flex \
        libtool \
        m4 \
    && rm -rf /var/lib/apt/lists/*

# install asnc1c
WORKDIR /setup
RUN git clone https://github.com/mouse07410/asn1c.git
WORKDIR /setup/asn1c
ARG ASN1C_COMMIT=9ac139f00f942b55d9961c7b9facbc1821aacb73
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
RUN echo "asn1c \$(find /input -name '*.asn' | sort) -fcompound-names -no-gen-BER --no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER" > /asn1c.sh
CMD ["/bin/bash", "/asn1c.sh"]
