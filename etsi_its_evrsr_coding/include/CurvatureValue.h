/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/cdd/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-PER`
 */

#ifndef	_CurvatureValue_H_
#define	_CurvatureValue_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CurvatureValue {
	CurvatureValue_straight	= 0,
	CurvatureValue_reciprocalOf1MeterRadiusToRight	= -30000,
	CurvatureValue_reciprocalOf1MeterRadiusToLeft	= 30000,
	CurvatureValue_unavailable	= 30001
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
oer_type_decoder_f CurvatureValue_decode_oer;
oer_type_encoder_f CurvatureValue_encode_oer;
per_type_decoder_f CurvatureValue_decode_uper;
per_type_encoder_f CurvatureValue_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _CurvatureValue_H_ */
#include <asn_internal.h>
