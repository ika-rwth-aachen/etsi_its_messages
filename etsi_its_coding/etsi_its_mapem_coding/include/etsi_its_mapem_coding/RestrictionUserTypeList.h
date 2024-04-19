/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE_OF.h>
#ifndef	_RestrictionUserTypeList_H_
#define	_RestrictionUserTypeList_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RestrictionUserType;

/* RestrictionUserTypeList */
typedef struct RestrictionUserTypeList {
	A_SEQUENCE_OF(struct RestrictionUserType) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RestrictionUserTypeList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RestrictionUserTypeList;
extern asn_SET_OF_specifics_t asn_SPC_RestrictionUserTypeList_specs_1;
extern asn_TYPE_member_t asn_MBR_RestrictionUserTypeList_1[1];
extern asn_per_constraints_t asn_PER_type_RestrictionUserTypeList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_coding/RestrictionUserType.h"

#endif	/* _RestrictionUserTypeList_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>