/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/NativeInteger.h>
#ifndef	_ValidityDuration_H_
#define	_ValidityDuration_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ValidityDuration {
	ValidityDuration_timeOfDetection	= 0,
	ValidityDuration_oneSecondAfterDetection	= 1
} e_ValidityDuration;

/* ValidityDuration */
typedef long	 ValidityDuration_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ValidityDuration;
asn_struct_free_f ValidityDuration_free;
asn_struct_print_f ValidityDuration_print;
asn_constr_check_f ValidityDuration_constraint;
ber_type_decoder_f ValidityDuration_decode_ber;
der_type_encoder_f ValidityDuration_encode_der;
xer_type_decoder_f ValidityDuration_decode_xer;
xer_type_encoder_f ValidityDuration_encode_xer;
jer_type_encoder_f ValidityDuration_encode_jer;
oer_type_decoder_f ValidityDuration_decode_oer;
oer_type_encoder_f ValidityDuration_encode_oer;
per_type_decoder_f ValidityDuration_decode_uper;
per_type_encoder_f ValidityDuration_encode_uper;
per_type_decoder_f ValidityDuration_decode_aper;
per_type_encoder_f ValidityDuration_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ValidityDuration_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
