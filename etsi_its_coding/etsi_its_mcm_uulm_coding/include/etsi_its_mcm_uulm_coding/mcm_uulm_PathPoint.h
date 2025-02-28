/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_PathPoint_H_
#define	_mcm_uulm_PathPoint_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_DeltaReferencePosition.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_PathDeltaTime.h"
#include <etsi_its_mcm_uulm_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mcm_uulm_PathPoint */
typedef struct mcm_uulm_PathPoint {
	mcm_uulm_DeltaReferencePosition_t	 pathPosition;
	mcm_uulm_PathDeltaTime_t	*pathDeltaTime;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_PathPoint_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_PathPoint;
extern asn_SEQUENCE_specifics_t asn_SPC_mcm_uulm_PathPoint_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_uulm_PathPoint_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_PathPoint_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
