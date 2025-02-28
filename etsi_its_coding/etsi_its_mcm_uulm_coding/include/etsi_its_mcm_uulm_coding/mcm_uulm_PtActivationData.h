/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_PtActivationData_H_
#define	_mcm_uulm_PtActivationData_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mcm_uulm_PtActivationData */
typedef OCTET_STRING_t	 mcm_uulm_PtActivationData_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_uulm_PtActivationData_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_PtActivationData;
asn_struct_free_f mcm_uulm_PtActivationData_free;
asn_struct_print_f mcm_uulm_PtActivationData_print;
asn_constr_check_f mcm_uulm_PtActivationData_constraint;
per_type_decoder_f mcm_uulm_PtActivationData_decode_uper;
per_type_encoder_f mcm_uulm_PtActivationData_encode_uper;
per_type_decoder_f mcm_uulm_PtActivationData_decode_aper;
per_type_encoder_f mcm_uulm_PtActivationData_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_PtActivationData_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
