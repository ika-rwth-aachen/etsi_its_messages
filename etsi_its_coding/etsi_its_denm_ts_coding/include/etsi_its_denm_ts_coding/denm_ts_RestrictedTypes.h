/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_RestrictedTypes_H_
#define	_denm_ts_RestrictedTypes_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_ts_coding/denm_ts_StationType.h"
#include <etsi_its_denm_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_ts_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* denm_ts_RestrictedTypes */
typedef struct denm_ts_RestrictedTypes {
	A_SEQUENCE_OF(denm_ts_StationType_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_ts_RestrictedTypes_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_RestrictedTypes;
extern asn_SET_OF_specifics_t asn_SPC_denm_ts_RestrictedTypes_specs_1;
extern asn_TYPE_member_t asn_MBR_denm_ts_RestrictedTypes_1[1];
extern asn_per_constraints_t asn_PER_type_denm_ts_RestrictedTypes_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_ts_RestrictedTypes_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
