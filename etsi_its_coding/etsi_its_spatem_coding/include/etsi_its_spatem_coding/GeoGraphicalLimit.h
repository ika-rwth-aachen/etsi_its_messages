/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AVIAEINumberingAndDataStructures"
 * 	found in "/input/ISO14816_AVIAEINumberingAndDataStructures.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/BIT_STRING.h>
#ifndef	_GeoGraphicalLimit_H_
#define	_GeoGraphicalLimit_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum GeoGraphicalLimit {
	GeoGraphicalLimit_globalRestriction	= 0,
	GeoGraphicalLimit_regionalRestriction	= 1,
	GeoGraphicalLimit_nationalRestriction	= 2,
	GeoGraphicalLimit_district	= 3,
	GeoGraphicalLimit_issuerCoverageRestriction	= 4,
	GeoGraphicalLimit_reservedForCEN1	= 5,
	GeoGraphicalLimit_reservedForCEN2	= 6,
	GeoGraphicalLimit_issuerSpecificRestriction	= 7
} e_GeoGraphicalLimit;

/* GeoGraphicalLimit */
typedef BIT_STRING_t	 GeoGraphicalLimit_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_GeoGraphicalLimit_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_GeoGraphicalLimit;
asn_struct_free_f GeoGraphicalLimit_free;
asn_struct_print_f GeoGraphicalLimit_print;
asn_constr_check_f GeoGraphicalLimit_constraint;
ber_type_decoder_f GeoGraphicalLimit_decode_ber;
der_type_encoder_f GeoGraphicalLimit_encode_der;
xer_type_decoder_f GeoGraphicalLimit_decode_xer;
xer_type_encoder_f GeoGraphicalLimit_encode_xer;
jer_type_encoder_f GeoGraphicalLimit_encode_jer;
oer_type_decoder_f GeoGraphicalLimit_decode_oer;
oer_type_encoder_f GeoGraphicalLimit_encode_oer;
per_type_decoder_f GeoGraphicalLimit_decode_uper;
per_type_encoder_f GeoGraphicalLimit_encode_uper;
per_type_decoder_f GeoGraphicalLimit_decode_aper;
per_type_encoder_f GeoGraphicalLimit_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _GeoGraphicalLimit_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
