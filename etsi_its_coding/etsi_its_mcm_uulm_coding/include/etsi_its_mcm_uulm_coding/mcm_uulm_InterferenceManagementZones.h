/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_InterferenceManagementZones_H_
#define	_mcm_uulm_InterferenceManagementZones_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mcm_uulm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mcm_uulm_InterferenceManagementZone;

/* mcm_uulm_InterferenceManagementZones */
typedef struct mcm_uulm_InterferenceManagementZones {
	A_SEQUENCE_OF(struct mcm_uulm_InterferenceManagementZone) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_InterferenceManagementZones_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_InterferenceManagementZones;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_InterferenceManagementZone.h"

#endif	/* _mcm_uulm_InterferenceManagementZones_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
