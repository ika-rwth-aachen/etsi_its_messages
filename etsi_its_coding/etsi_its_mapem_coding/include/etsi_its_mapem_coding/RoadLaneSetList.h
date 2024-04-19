/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE_OF.h>
#ifndef	_RoadLaneSetList_H_
#define	_RoadLaneSetList_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct GenericLane;

/* RoadLaneSetList */
typedef struct RoadLaneSetList {
	A_SEQUENCE_OF(struct GenericLane) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadLaneSetList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadLaneSetList;
extern asn_SET_OF_specifics_t asn_SPC_RoadLaneSetList_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadLaneSetList_1[1];
extern asn_per_constraints_t asn_PER_type_RoadLaneSetList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_coding/GenericLane.h"

#endif	/* _RoadLaneSetList_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>