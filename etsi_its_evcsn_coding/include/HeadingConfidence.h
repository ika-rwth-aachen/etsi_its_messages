/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/cdd/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-PER`
 */

#ifndef	_HeadingConfidence_H_
#define	_HeadingConfidence_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum HeadingConfidence {
	HeadingConfidence_equalOrWithinZeroPointOneDegree	= 1,
	HeadingConfidence_equalOrWithinOneDegree	= 10,
	HeadingConfidence_outOfRange	= 126,
	HeadingConfidence_unavailable	= 127
} e_HeadingConfidence;

/* HeadingConfidence */
typedef long	 HeadingConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_HeadingConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_HeadingConfidence;
asn_struct_free_f HeadingConfidence_free;
asn_struct_print_f HeadingConfidence_print;
asn_constr_check_f HeadingConfidence_constraint;
ber_type_decoder_f HeadingConfidence_decode_ber;
der_type_encoder_f HeadingConfidence_encode_der;
xer_type_decoder_f HeadingConfidence_decode_xer;
xer_type_encoder_f HeadingConfidence_encode_xer;
oer_type_decoder_f HeadingConfidence_decode_oer;
oer_type_encoder_f HeadingConfidence_encode_oer;
per_type_decoder_f HeadingConfidence_decode_uper;
per_type_encoder_f HeadingConfidence_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _HeadingConfidence_H_ */
#include <asn_internal.h>
