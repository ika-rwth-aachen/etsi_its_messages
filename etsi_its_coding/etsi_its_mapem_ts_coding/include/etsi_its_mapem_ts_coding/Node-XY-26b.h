/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/Offset-B13.h"
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#ifndef	_Node_XY_26b_H_
#define	_Node_XY_26b_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Node-XY-26b */
typedef struct Node_XY_26b {
	Offset_B13_t	 x;
	Offset_B13_t	 y;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Node_XY_26b_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Node_XY_26b;
extern asn_SEQUENCE_specifics_t asn_SPC_Node_XY_26b_specs_1;
extern asn_TYPE_member_t asn_MBR_Node_XY_26b_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Node_XY_26b_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
