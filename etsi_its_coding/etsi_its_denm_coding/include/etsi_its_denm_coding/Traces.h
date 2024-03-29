/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/constr_SEQUENCE_OF.h>
#ifndef	_Traces_H_
#define	_Traces_H_


#include <etsi_its_denm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PathHistory;

/* Traces */
typedef struct Traces {
	A_SEQUENCE_OF(struct PathHistory) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Traces_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Traces;
extern asn_SET_OF_specifics_t asn_SPC_Traces_specs_1;
extern asn_TYPE_member_t asn_MBR_Traces_1[1];
extern asn_per_constraints_t asn_PER_type_Traces_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_denm_coding/PathHistory.h"

#endif	/* _Traces_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
