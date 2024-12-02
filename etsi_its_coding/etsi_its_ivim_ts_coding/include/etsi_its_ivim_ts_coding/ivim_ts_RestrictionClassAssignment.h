/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_RestrictionClassAssignment_H_
#define	_ivim_ts_RestrictionClassAssignment_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_RestrictionClassID.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RestrictionUserTypeList.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_RestrictionClassAssignment */
typedef struct ivim_ts_RestrictionClassAssignment {
	ivim_ts_RestrictionClassID_t	 id;
	ivim_ts_RestrictionUserTypeList_t	 users;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_RestrictionClassAssignment_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_RestrictionClassAssignment;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_RestrictionClassAssignment_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_RestrictionClassAssignment_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_RestrictionClassAssignment_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
