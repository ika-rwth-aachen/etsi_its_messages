/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_denm_coding/NativeInteger.h>
#ifndef	_AltitudeValue_H_
#define	_AltitudeValue_H_


#include <etsi_its_denm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AltitudeValue {
	AltitudeValue_referenceEllipsoidSurface	= 0,
	AltitudeValue_oneCentimeter	= 1,
	AltitudeValue_unavailable	= 800001
} e_AltitudeValue;

/* AltitudeValue */
typedef long	 AltitudeValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_AltitudeValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_AltitudeValue;
asn_struct_free_f AltitudeValue_free;
asn_struct_print_f AltitudeValue_print;
asn_constr_check_f AltitudeValue_constraint;
ber_type_decoder_f AltitudeValue_decode_ber;
der_type_encoder_f AltitudeValue_encode_der;
xer_type_decoder_f AltitudeValue_decode_xer;
xer_type_encoder_f AltitudeValue_encode_xer;
jer_type_encoder_f AltitudeValue_encode_jer;
oer_type_decoder_f AltitudeValue_decode_oer;
oer_type_encoder_f AltitudeValue_encode_oer;
per_type_decoder_f AltitudeValue_decode_uper;
per_type_encoder_f AltitudeValue_encode_uper;
per_type_decoder_f AltitudeValue_decode_aper;
per_type_encoder_f AltitudeValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _AltitudeValue_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
