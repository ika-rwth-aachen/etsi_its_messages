/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/NativeInteger.h>
#ifndef	_SpeedConfidence_H_
#define	_SpeedConfidence_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SpeedConfidence {
	SpeedConfidence_equalOrWithinOneCentimeterPerSec	= 1,
	SpeedConfidence_equalOrWithinOneMeterPerSec	= 100,
	SpeedConfidence_outOfRange	= 126,
	SpeedConfidence_unavailable	= 127
} e_SpeedConfidence;

/* SpeedConfidence */
typedef long	 SpeedConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_SpeedConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_SpeedConfidence;
asn_struct_free_f SpeedConfidence_free;
asn_struct_print_f SpeedConfidence_print;
asn_constr_check_f SpeedConfidence_constraint;
ber_type_decoder_f SpeedConfidence_decode_ber;
der_type_encoder_f SpeedConfidence_encode_der;
xer_type_decoder_f SpeedConfidence_decode_xer;
xer_type_encoder_f SpeedConfidence_encode_xer;
jer_type_encoder_f SpeedConfidence_encode_jer;
oer_type_decoder_f SpeedConfidence_decode_oer;
oer_type_encoder_f SpeedConfidence_encode_oer;
per_type_decoder_f SpeedConfidence_decode_uper;
per_type_encoder_f SpeedConfidence_encode_uper;
per_type_decoder_f SpeedConfidence_decode_aper;
per_type_encoder_f SpeedConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SpeedConfidence_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>