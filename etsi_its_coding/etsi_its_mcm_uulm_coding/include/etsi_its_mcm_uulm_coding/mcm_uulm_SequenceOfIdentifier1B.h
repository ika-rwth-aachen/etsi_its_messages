/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_SequenceOfIdentifier1B_H_
#define	_mcm_uulm_SequenceOfIdentifier1B_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_Identifier1B.h"
#include <etsi_its_mcm_uulm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mcm_uulm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mcm_uulm_SequenceOfIdentifier1B */
typedef struct mcm_uulm_SequenceOfIdentifier1B {
	A_SEQUENCE_OF(mcm_uulm_Identifier1B_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_SequenceOfIdentifier1B_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_SequenceOfIdentifier1B;
extern asn_SET_OF_specifics_t asn_SPC_mcm_uulm_SequenceOfIdentifier1B_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_uulm_SequenceOfIdentifier1B_1[1];
extern asn_per_constraints_t asn_PER_type_mcm_uulm_SequenceOfIdentifier1B_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_SequenceOfIdentifier1B_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
