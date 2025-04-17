/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "MCM-PDU-Descriptions"
 * 	found in "/input/TS103561_LUKAS_MCM.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_DesiredRoute_H_
#define	_mcm_uulm_DesiredRoute_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mcm_uulm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mcm_uulm_Waypoint;

/* mcm_uulm_DesiredRoute */
typedef struct mcm_uulm_DesiredRoute {
	A_SEQUENCE_OF(struct mcm_uulm_Waypoint) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_DesiredRoute_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_DesiredRoute;
extern asn_SET_OF_specifics_t asn_SPC_mcm_uulm_DesiredRoute_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_uulm_DesiredRoute_1[1];
extern asn_per_constraints_t asn_PER_type_mcm_uulm_DesiredRoute_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_Waypoint.h"

#endif	/* _mcm_uulm_DesiredRoute_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
