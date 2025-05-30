/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_PathHistory_H_
#define	_mcm_uulm_PathHistory_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mcm_uulm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mcm_uulm_PathPoint;

/* mcm_uulm_PathHistory */
typedef struct mcm_uulm_PathHistory {
	A_SEQUENCE_OF(struct mcm_uulm_PathPoint) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_PathHistory_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_PathHistory;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_PathPoint.h"

#endif	/* _mcm_uulm_PathHistory_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
