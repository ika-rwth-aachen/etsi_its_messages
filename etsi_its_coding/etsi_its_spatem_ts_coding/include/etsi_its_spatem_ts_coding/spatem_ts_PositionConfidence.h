/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_spatem_ts_PositionConfidence_H_
#define	_spatem_ts_PositionConfidence_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum spatem_ts_PositionConfidence {
	spatem_ts_PositionConfidence_unavailable	= 0,
	spatem_ts_PositionConfidence_a500m	= 1,
	spatem_ts_PositionConfidence_a200m	= 2,
	spatem_ts_PositionConfidence_a100m	= 3,
	spatem_ts_PositionConfidence_a50m	= 4,
	spatem_ts_PositionConfidence_a20m	= 5,
	spatem_ts_PositionConfidence_a10m	= 6,
	spatem_ts_PositionConfidence_a5m	= 7,
	spatem_ts_PositionConfidence_a2m	= 8,
	spatem_ts_PositionConfidence_a1m	= 9,
	spatem_ts_PositionConfidence_a50cm	= 10,
	spatem_ts_PositionConfidence_a20cm	= 11,
	spatem_ts_PositionConfidence_a10cm	= 12,
	spatem_ts_PositionConfidence_a5cm	= 13,
	spatem_ts_PositionConfidence_a2cm	= 14,
	spatem_ts_PositionConfidence_a1cm	= 15
} e_spatem_ts_PositionConfidence;

/* spatem_ts_PositionConfidence */
typedef long	 spatem_ts_PositionConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_spatem_ts_PositionConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_PositionConfidence;
extern const asn_INTEGER_specifics_t asn_SPC_spatem_ts_PositionConfidence_specs_1;
asn_struct_free_f spatem_ts_PositionConfidence_free;
asn_struct_print_f spatem_ts_PositionConfidence_print;
asn_constr_check_f spatem_ts_PositionConfidence_constraint;
per_type_decoder_f spatem_ts_PositionConfidence_decode_uper;
per_type_encoder_f spatem_ts_PositionConfidence_encode_uper;
per_type_decoder_f spatem_ts_PositionConfidence_decode_aper;
per_type_encoder_f spatem_ts_PositionConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_PositionConfidence_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>