/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_spatem_coding/PositionConfidence.h"
#include "etsi_its_spatem_coding/ElevationConfidence.h"
#include <etsi_its_spatem_coding/constr_SEQUENCE.h>
#ifndef	_PositionConfidenceSet_H_
#define	_PositionConfidenceSet_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PositionConfidenceSet */
typedef struct PositionConfidenceSet {
	PositionConfidence_t	 pos;
	ElevationConfidence_t	 elevation;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PositionConfidenceSet_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PositionConfidenceSet;
extern asn_SEQUENCE_specifics_t asn_SPC_PositionConfidenceSet_specs_1;
extern asn_TYPE_member_t asn_MBR_PositionConfidenceSet_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _PositionConfidenceSet_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
