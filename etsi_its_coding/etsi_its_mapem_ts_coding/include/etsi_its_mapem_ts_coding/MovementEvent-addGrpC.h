/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/ExceptionalCondition.h"
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#ifndef	_MovementEvent_addGrpC_H_
#define	_MovementEvent_addGrpC_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MovementEvent-addGrpC */
typedef struct MovementEvent_addGrpC {
	ExceptionalCondition_t	*stateChangeReason;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MovementEvent_addGrpC_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MovementEvent_addGrpC;
extern asn_SEQUENCE_specifics_t asn_SPC_MovementEvent_addGrpC_specs_1;
extern asn_TYPE_member_t asn_MBR_MovementEvent_addGrpC_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _MovementEvent_addGrpC_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
