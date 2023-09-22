# === VARIABLES ================================================================
MAKEFILE_DIR := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

ID_USER = $(shell id -u ${USER})
ID_GROUP = $(shell id -g ${USER})

# === Compile ====================================================================

all: 
	$(MAKE) -s cam

cam:
	@docker run --volume ${MAKEFILE_DIR}/../../asn1/reduced/cam:/input --volume ${MAKEFILE_DIR}/../../../etsi_its_coding/etsi_its_cam_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/fb-fi/definitions/etsi_its_messages/asn1c:latest
	@./postprocessAsn1cIncludes.py ../../../etsi_its_coding/etsi_its_cam_coding