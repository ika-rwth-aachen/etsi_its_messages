# === VARIABLES ================================================================
MAKEFILE_DIR := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

ID_USER = $(shell id -u ${USER})
ID_GROUP = $(shell id -g ${USER})

# === Compile ====================================================================

all: cam cdd denm evcsn evrsr ivim mapem poti rtcmem spatem srem ssem tistpg

cam:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/cam_en302637_2:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_cam:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

cdd:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/cdd_ts102894_2:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_cdd:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

denm:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/denm_en302637_3:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_denm:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

evcsn:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/evcsn-ts101556_1:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_evcsn:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

evrsr:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/evrsr_ts101556_3:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_evrsr:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

ivim:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/IVIM-PDU-Descriptions.asn:/input/IVIM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../etsi_its_coding_ivim:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

mapem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/MAPEM-PDU-Descriptions.asn:/input/MAPEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../etsi_its_coding_mapem/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

poti:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/poti_en302890_2:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_poti:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

rtcmem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/RTCMEM-PDU-Descriptions.asn:/input/RTCMEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../etsi_its_coding_rtcmem:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

spatem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/SPATEM-PDU-Descriptions.asn:/input/SPATEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/dependencies:/input/dependencies --volume ${MAKEFILE_DIR}/../../etsi_its_coding_spatem/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

srem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/SREM-PDU-Descriptions.asn:/input/SREM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../etsi_its_coding_srem:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

ssem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/SSEM-PDU-Descriptions.asn:/input/SSEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../etsi_its_coding_ssem:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

tistpg:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/tistpg_ts101556_2:/input --volume ${MAKEFILE_DIR}/../../etsi_its_coding_tstpg:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest