/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_ConfidenceLevel_H_
#define	_mcm_uulm_ConfidenceLevel_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_ConfidenceLevel {
	mcm_uulm_ConfidenceLevel_unavailable	= 101
} e_mcm_uulm_ConfidenceLevel;

/* mcm_uulm_ConfidenceLevel */
typedef long	 mcm_uulm_ConfidenceLevel_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_uulm_ConfidenceLevel_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_ConfidenceLevel;
asn_struct_free_f mcm_uulm_ConfidenceLevel_free;
asn_struct_print_f mcm_uulm_ConfidenceLevel_print;
asn_constr_check_f mcm_uulm_ConfidenceLevel_constraint;
per_type_decoder_f mcm_uulm_ConfidenceLevel_decode_uper;
per_type_encoder_f mcm_uulm_ConfidenceLevel_encode_uper;
per_type_decoder_f mcm_uulm_ConfidenceLevel_decode_aper;
per_type_encoder_f mcm_uulm_ConfidenceLevel_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_ConfidenceLevel_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
