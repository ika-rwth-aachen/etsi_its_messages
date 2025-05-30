/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AVIAEINumberingAndDataStructures"
 * 	found in "/input/ISO14816_AVIAEINumberingAndDataStructures.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_GeoGraphicalLimit_H_
#define	_spatem_ts_GeoGraphicalLimit_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum spatem_ts_GeoGraphicalLimit {
	spatem_ts_GeoGraphicalLimit_globalRestriction	= 0,
	spatem_ts_GeoGraphicalLimit_regionalRestriction	= 1,
	spatem_ts_GeoGraphicalLimit_nationalRestriction	= 2,
	spatem_ts_GeoGraphicalLimit_district	= 3,
	spatem_ts_GeoGraphicalLimit_issuerCoverageRestriction	= 4,
	spatem_ts_GeoGraphicalLimit_reservedForCEN1	= 5,
	spatem_ts_GeoGraphicalLimit_reservedForCEN2	= 6,
	spatem_ts_GeoGraphicalLimit_issuerSpecificRestriction	= 7
} e_spatem_ts_GeoGraphicalLimit;

/* spatem_ts_GeoGraphicalLimit */
typedef BIT_STRING_t	 spatem_ts_GeoGraphicalLimit_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_spatem_ts_GeoGraphicalLimit_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_GeoGraphicalLimit;
asn_struct_free_f spatem_ts_GeoGraphicalLimit_free;
asn_struct_print_f spatem_ts_GeoGraphicalLimit_print;
asn_constr_check_f spatem_ts_GeoGraphicalLimit_constraint;
jer_type_encoder_f spatem_ts_GeoGraphicalLimit_encode_jer;
per_type_decoder_f spatem_ts_GeoGraphicalLimit_decode_uper;
per_type_encoder_f spatem_ts_GeoGraphicalLimit_encode_uper;
per_type_decoder_f spatem_ts_GeoGraphicalLimit_decode_aper;
per_type_encoder_f spatem_ts_GeoGraphicalLimit_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_GeoGraphicalLimit_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
