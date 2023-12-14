/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/NativeInteger.h>
#ifndef	_CurvatureValue_H_
#define	_CurvatureValue_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CurvatureValue {
	CurvatureValue_straight	= 0,
	CurvatureValue_unavailable	= 1023
} e_CurvatureValue;

/* CurvatureValue */
typedef long	 CurvatureValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_CurvatureValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_CurvatureValue;
asn_struct_free_f CurvatureValue_free;
asn_struct_print_f CurvatureValue_print;
asn_constr_check_f CurvatureValue_constraint;
ber_type_decoder_f CurvatureValue_decode_ber;
der_type_encoder_f CurvatureValue_encode_der;
xer_type_decoder_f CurvatureValue_decode_xer;
xer_type_encoder_f CurvatureValue_encode_xer;
jer_type_encoder_f CurvatureValue_encode_jer;
oer_type_decoder_f CurvatureValue_decode_oer;
oer_type_encoder_f CurvatureValue_encode_oer;
per_type_decoder_f CurvatureValue_decode_uper;
per_type_encoder_f CurvatureValue_encode_uper;
per_type_decoder_f CurvatureValue_decode_aper;
per_type_encoder_f CurvatureValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _CurvatureValue_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
