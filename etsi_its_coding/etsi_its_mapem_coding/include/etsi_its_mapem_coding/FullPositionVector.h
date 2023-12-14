/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_coding/Longitude.h"
#include "etsi_its_mapem_coding/Latitude.h"
#include "etsi_its_mapem_coding/Elevation.h"
#include "etsi_its_mapem_coding/HeadingDSRC.h"
#include "etsi_its_mapem_coding/TimeConfidence.h"
#include <etsi_its_mapem_coding/constr_SEQUENCE.h>
#ifndef	_FullPositionVector_H_
#define	_FullPositionVector_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct DDateTime;
struct TransmissionAndSpeed;
struct PositionalAccuracy;
struct PositionConfidenceSet;
struct SpeedandHeadingandThrottleConfidence;

/* FullPositionVector */
typedef struct FullPositionVector {
	struct DDateTime	*utcTime;	/* OPTIONAL */
	Longitude_t	 Long;
	Latitude_t	 lat;
	Elevation_t	*elevation;	/* OPTIONAL */
	HeadingDSRC_t	*heading;	/* OPTIONAL */
	struct TransmissionAndSpeed	*speed;	/* OPTIONAL */
	struct PositionalAccuracy	*posAccuracy;	/* OPTIONAL */
	TimeConfidence_t	*timeConfidence;	/* OPTIONAL */
	struct PositionConfidenceSet	*posConfidence;	/* OPTIONAL */
	struct SpeedandHeadingandThrottleConfidence	*speedConfidence;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} FullPositionVector_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_FullPositionVector;
extern asn_SEQUENCE_specifics_t asn_SPC_FullPositionVector_specs_1;
extern asn_TYPE_member_t asn_MBR_FullPositionVector_1[10];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_coding/DDateTime.h"
#include "etsi_its_mapem_coding/TransmissionAndSpeed.h"
#include "etsi_its_mapem_coding/PositionalAccuracy.h"
#include "etsi_its_mapem_coding/PositionConfidenceSet.h"
#include "etsi_its_mapem_coding/SpeedandHeadingandThrottleConfidence.h"

#endif	/* _FullPositionVector_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
