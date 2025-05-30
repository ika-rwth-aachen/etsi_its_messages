/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "/input/CAM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_RoadWorksContainerBasic_H_
#define	_cam_ts_RoadWorksContainerBasic_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_ts_coding/cam_ts_RoadworksSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_LightBarSirenInUse.h"
#include <etsi_its_cam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cam_ts_ClosedLanes;

/* cam_ts_RoadWorksContainerBasic */
typedef struct cam_ts_RoadWorksContainerBasic {
	cam_ts_RoadworksSubCauseCode_t	*roadworksSubCauseCode;	/* OPTIONAL */
	cam_ts_LightBarSirenInUse_t	 lightBarSirenInUse;
	struct cam_ts_ClosedLanes	*closedLanes;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_RoadWorksContainerBasic_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_RoadWorksContainerBasic;
extern asn_SEQUENCE_specifics_t asn_SPC_cam_ts_RoadWorksContainerBasic_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_RoadWorksContainerBasic_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cam_ts_coding/cam_ts_ClosedLanes.h"

#endif	/* _cam_ts_RoadWorksContainerBasic_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
