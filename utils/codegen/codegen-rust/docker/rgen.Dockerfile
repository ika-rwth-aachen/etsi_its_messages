# docker build -t ghcr.io/ika-rwth-aachen/etsi_its_messages:rgen -f rgen.Dockerfile ..

FROM ubuntu:22.04

# install essentials
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        git \
        curl \
        tar \
        wget \
    && rm -rf /var/lib/apt/lists/*

# install Rust/Cargo
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# Compile .msg, conversion headers generators
WORKDIR /setup
COPY rgen/ rgen/
WORKDIR /setup/rgen
RUN cargo build --release && \
    mv target/release/asn1-to-ros-msgs /usr/local/bin/asn1-to-ros-msgs && \
    mv target/release/asn1-to-ros-conversion-headers /usr/local/bin/asn1-to-ros-conversion-headers

# cleanup
WORKDIR /
RUN rm -rf /setup

# command
RUN mkdir input
RUN mkdir output
WORKDIR /output
RUN echo "\
generator=\$1\n\
pdu=\$2\n\
case \$generator in\n\
    'msgs')\n\
        asn1-to-ros-msgs -o . \$(find /input -name '*.asn' | sort)\n\
        ;;\n\
    'conversion-headers')\n\
        asn1-to-ros-conversion-headers -o . -p \$pdu \$(find /input -name '*.asn' | sort)\n\
        ;;\n\
    *)\n\
        echo 'Unknown generator \$generator'\n\
        exit 1\n\
        ;;\n\
esac\n\
" > /rgen.sh
ENTRYPOINT ["/bin/bash", "/rgen.sh"]
CMD ["msgs", "test"]
