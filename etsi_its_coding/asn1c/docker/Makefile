# === VARIABLES ================================================================
MAKEFILE_DIR := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

ID_USER = $(shell id -u ${USER})
ID_GROUP = $(shell id -g ${USER})

# === Compile ====================================================================

all: 
	$(MAKE) -s cam 
	$(MAKE) -s cdd 
	$(MAKE) -s denm 
	$(MAKE) -s evcsn 
	$(MAKE) -s evrsr 
	$(MAKE) -s ivim 
	$(MAKE) -s mapem 
	$(MAKE) -s rtcmem 
	$(MAKE) -s spatem 
	$(MAKE) -s srem 
	$(MAKE) -s ssem 

cam:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/cam_en302637_2:/input --volume ${MAKEFILE_DIR}/../../../etsi_its_cam_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

cdd:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/cdd_ts102894_2:/input --volume ${MAKEFILE_DIR}/../../../etsi_its_cdd_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

denm:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/denm_en302637_3:/input --volume ${MAKEFILE_DIR}/../../../etsi_its_denm_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

evcsn:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/evcsn-ts101556_1:/input --volume ${MAKEFILE_DIR}/../../../etsi_its_evcsn_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

evrsr:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/evrsr_ts101556_3:/input --volume ${MAKEFILE_DIR}/../../../etsi_its_evrsr_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

ivim:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/IVIM-PDU-Descriptions.asn:/input/IVIM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../etsi_its_ivim_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

mapem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/MAPEM-PDU-Descriptions.asn:/input/MAPEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../etsi_its_mapem_coding/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

rtcmem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/RTCMEM-PDU-Descriptions.asn:/input/RTCMEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../etsi_its_rtcmem_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

spatem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/SPATEM-PDU-Descriptions.asn:/input/SPATEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/dependencies:/input/dependencies --volume ${MAKEFILE_DIR}/../../../etsi_its_spatem_coding/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

srem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/SREM-PDU-Descriptions.asn:/input/SREM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../etsi_its_srem_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest

ssem:
	@docker run --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/SSEM-PDU-Descriptions.asn:/input/SSEM-PDU-Descriptions.asn --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/cdd:/input/cdd --volume ${MAKEFILE_DIR}/../../../ETSI-ITS-ASN1/is_ts103301/iso-patched:/input/iso-patched --volume ${MAKEFILE_DIR}/../../../etsi_its_ssem_coding:/output -u ${ID_USER}:${ID_GROUP} gitlab.ika.rwth-aachen.de:5050/automated-driving/ros_etsi_its_messages/asn1c:latest