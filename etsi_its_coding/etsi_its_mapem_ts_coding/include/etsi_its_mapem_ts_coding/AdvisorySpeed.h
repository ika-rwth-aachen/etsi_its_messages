/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/AdvisorySpeedType.h"
#include "etsi_its_mapem_ts_coding/SpeedAdvice.h"
#include "etsi_its_mapem_ts_coding/SpeedConfidenceDSRC.h"
#include "etsi_its_mapem_ts_coding/ZoneLength.h"
#include "etsi_its_mapem_ts_coding/RestrictionClassID.h"
#include <etsi_its_mapem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#ifndef	_AdvisorySpeed_H_
#define	_AdvisorySpeed_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Reg_AdvisorySpeed;

/* AdvisorySpeed */
typedef struct AdvisorySpeed {
	AdvisorySpeedType_t	 type;
	SpeedAdvice_t	*speed;	/* OPTIONAL */
	SpeedConfidenceDSRC_t	*confidence;	/* OPTIONAL */
	ZoneLength_t	*distance;	/* OPTIONAL */
	RestrictionClassID_t	*Class;	/* OPTIONAL */
	struct AdvisorySpeed__regional {
		A_SEQUENCE_OF(struct Reg_AdvisorySpeed) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AdvisorySpeed_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AdvisorySpeed;
extern asn_SEQUENCE_specifics_t asn_SPC_AdvisorySpeed_specs_1;
extern asn_TYPE_member_t asn_MBR_AdvisorySpeed_1[6];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_ts_coding/RegionalExtension.h"

#endif	/* _AdvisorySpeed_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
