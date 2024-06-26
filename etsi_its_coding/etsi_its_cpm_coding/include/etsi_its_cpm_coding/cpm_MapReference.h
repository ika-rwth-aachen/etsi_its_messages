/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_MapReference_H_
#define	_cpm_MapReference_H_


#include <etsi_its_cpm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cpm_coding/cpm_RoadSegmentReferenceId.h"
#include "etsi_its_cpm_coding/cpm_IntersectionReferenceId.h"
#include <etsi_its_cpm_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cpm_MapReference_PR {
	cpm_MapReference_PR_NOTHING,	/* No components present */
	cpm_MapReference_PR_roadsegment,
	cpm_MapReference_PR_intersection
} cpm_MapReference_PR;

/* cpm_MapReference */
typedef struct cpm_MapReference {
	cpm_MapReference_PR present;
	union cpm_MapReference_u {
		cpm_RoadSegmentReferenceId_t	 roadsegment;
		cpm_IntersectionReferenceId_t	 intersection;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_MapReference_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_MapReference;
extern asn_CHOICE_specifics_t asn_SPC_cpm_MapReference_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_MapReference_1[2];
extern asn_per_constraints_t asn_PER_type_cpm_MapReference_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_MapReference_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
