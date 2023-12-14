/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/BIT_STRING.h>
#ifndef	_SpecialTransportType_H_
#define	_SpecialTransportType_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SpecialTransportType {
	SpecialTransportType_heavyLoad	= 0,
	SpecialTransportType_excessWidth	= 1,
	SpecialTransportType_excessLength	= 2,
	SpecialTransportType_excessHeight	= 3
} e_SpecialTransportType;

/* SpecialTransportType */
typedef BIT_STRING_t	 SpecialTransportType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SpecialTransportType;
asn_struct_free_f SpecialTransportType_free;
asn_struct_print_f SpecialTransportType_print;
asn_constr_check_f SpecialTransportType_constraint;
ber_type_decoder_f SpecialTransportType_decode_ber;
der_type_encoder_f SpecialTransportType_encode_der;
xer_type_decoder_f SpecialTransportType_decode_xer;
xer_type_encoder_f SpecialTransportType_encode_xer;
jer_type_encoder_f SpecialTransportType_encode_jer;
oer_type_decoder_f SpecialTransportType_decode_oer;
oer_type_encoder_f SpecialTransportType_encode_oer;
per_type_decoder_f SpecialTransportType_decode_uper;
per_type_encoder_f SpecialTransportType_encode_uper;
per_type_decoder_f SpecialTransportType_decode_aper;
per_type_encoder_f SpecialTransportType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SpecialTransportType_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
