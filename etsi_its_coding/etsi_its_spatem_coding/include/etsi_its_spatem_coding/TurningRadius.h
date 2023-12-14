/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/NativeInteger.h>
#ifndef	_TurningRadius_H_
#define	_TurningRadius_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TurningRadius {
	TurningRadius_point4Meters	= 1,
	TurningRadius_unavailable	= 255
} e_TurningRadius;

/* TurningRadius */
typedef long	 TurningRadius_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TurningRadius;
asn_struct_free_f TurningRadius_free;
asn_struct_print_f TurningRadius_print;
asn_constr_check_f TurningRadius_constraint;
ber_type_decoder_f TurningRadius_decode_ber;
der_type_encoder_f TurningRadius_encode_der;
xer_type_decoder_f TurningRadius_decode_xer;
xer_type_encoder_f TurningRadius_encode_xer;
jer_type_encoder_f TurningRadius_encode_jer;
oer_type_decoder_f TurningRadius_decode_oer;
oer_type_encoder_f TurningRadius_encode_oer;
per_type_decoder_f TurningRadius_decode_uper;
per_type_encoder_f TurningRadius_encode_uper;
per_type_decoder_f TurningRadius_decode_aper;
per_type_encoder_f TurningRadius_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TurningRadius_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
