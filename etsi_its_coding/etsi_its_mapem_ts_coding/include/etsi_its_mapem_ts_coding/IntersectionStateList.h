/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE_OF.h>
#ifndef	_IntersectionStateList_H_
#define	_IntersectionStateList_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct IntersectionState;

/* IntersectionStateList */
typedef struct IntersectionStateList {
	A_SEQUENCE_OF(struct IntersectionState) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntersectionStateList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntersectionStateList;
extern asn_SET_OF_specifics_t asn_SPC_IntersectionStateList_specs_1;
extern asn_TYPE_member_t asn_MBR_IntersectionStateList_1[1];
extern asn_per_constraints_t asn_PER_type_IntersectionStateList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_ts_coding/IntersectionState.h"

#endif	/* _IntersectionStateList_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
