FROM ubuntu:22.04

USER root

# Install git
RUN apt-get update && \
    apt-get install -y \
    git

# Install ASN1C dependencies
RUN apt-get update && \
    apt-get install -y \
    automake=1.15 \
    libtool \
    bison=2.x \
    flex && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/vlm/asn1c.git


