/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_StoredInformationType_H_
#define	_mcm_uulm_StoredInformationType_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_StoredInformationType {
	mcm_uulm_StoredInformationType_undefined	= 0,
	mcm_uulm_StoredInformationType_staticDb	= 1,
	mcm_uulm_StoredInformationType_dynamicDb	= 2,
	mcm_uulm_StoredInformationType_realTimeDb	= 3,
	mcm_uulm_StoredInformationType_map	= 4
} e_mcm_uulm_StoredInformationType;

/* mcm_uulm_StoredInformationType */
typedef BIT_STRING_t	 mcm_uulm_StoredInformationType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_uulm_StoredInformationType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_StoredInformationType;
asn_struct_free_f mcm_uulm_StoredInformationType_free;
asn_struct_print_f mcm_uulm_StoredInformationType_print;
asn_constr_check_f mcm_uulm_StoredInformationType_constraint;
per_type_decoder_f mcm_uulm_StoredInformationType_decode_uper;
per_type_encoder_f mcm_uulm_StoredInformationType_encode_uper;
per_type_decoder_f mcm_uulm_StoredInformationType_decode_aper;
per_type_encoder_f mcm_uulm_StoredInformationType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_StoredInformationType_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
