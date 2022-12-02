# ASN.1 Coding using asn1c

## Quick Start

Follow these steps to quickly generate your `*.h` and `*.c` files from a given set of `*.asn1` files.

### Pull docker image

```bash
docker login gitlab.ika.rwth-aachen.de:5050
docker pull gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest
```

### Run the container

_Make sure to use an absolute path for the `<local_input/output_folder>`!_

All ASN1 Files within the `<local_input_folder>` will be compiled by asn1c. The resulting `*.h`- and `.c`-files will be placed respecitvely in an `ìnclude` and `src` folder within the `<local_output_folder>`.

_Make sure that your ASN1 dependency-files are matching since duplicate-files will be ignored (only the file-version found first is used)!_

```bash
docker run --volume <local_input_folder>:/home/input --volume <local_output_folder>:/home/output gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest
```

### Alternative: Usage of makefile

We prepared a makefile to build specifc coding packages for each etsi message type.
Simply run one of the following commands from within the `etsi_its_coding/asn1c/docker` directory:

```bash
make <all/cam/denm/...>
```

The generated `*.c` and `*.h`-files will be placed into the specific `etsi_its_<...>_coding` directory.

### Build docker image locally

```bash
cd docker
docker build --tag gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest .
```