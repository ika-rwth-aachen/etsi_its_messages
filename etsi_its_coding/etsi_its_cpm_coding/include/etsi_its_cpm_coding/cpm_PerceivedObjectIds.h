/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PerceptionRegionContainer"
 * 	found in "/input/CPM-PerceptionRegionContainer.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_PerceivedObjectIds_H_
#define	_cpm_PerceivedObjectIds_H_


#include <etsi_its_cpm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cpm_coding/cpm_Identifier2B.h"
#include <etsi_its_cpm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_cpm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* cpm_PerceivedObjectIds */
typedef struct cpm_PerceivedObjectIds {
	A_SEQUENCE_OF(cpm_Identifier2B_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_PerceivedObjectIds_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_PerceivedObjectIds;
extern asn_SET_OF_specifics_t asn_SPC_cpm_PerceivedObjectIds_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_PerceivedObjectIds_1[1];
extern asn_per_constraints_t asn_PER_type_cpm_PerceivedObjectIds_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_PerceivedObjectIds_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
