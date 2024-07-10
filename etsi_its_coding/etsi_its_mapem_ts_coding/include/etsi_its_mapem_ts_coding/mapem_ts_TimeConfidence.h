/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mapem_ts_TimeConfidence_H_
#define	_mapem_ts_TimeConfidence_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_TimeConfidence {
	mapem_ts_TimeConfidence_unavailable	= 0,
	mapem_ts_TimeConfidence_time_100_000	= 1,
	mapem_ts_TimeConfidence_time_050_000	= 2,
	mapem_ts_TimeConfidence_time_020_000	= 3,
	mapem_ts_TimeConfidence_time_010_000	= 4,
	mapem_ts_TimeConfidence_time_002_000	= 5,
	mapem_ts_TimeConfidence_time_001_000	= 6,
	mapem_ts_TimeConfidence_time_000_500	= 7,
	mapem_ts_TimeConfidence_time_000_200	= 8,
	mapem_ts_TimeConfidence_time_000_100	= 9,
	mapem_ts_TimeConfidence_time_000_050	= 10,
	mapem_ts_TimeConfidence_time_000_020	= 11,
	mapem_ts_TimeConfidence_time_000_010	= 12,
	mapem_ts_TimeConfidence_time_000_005	= 13,
	mapem_ts_TimeConfidence_time_000_002	= 14,
	mapem_ts_TimeConfidence_time_000_001	= 15,
	mapem_ts_TimeConfidence_time_000_000_5	= 16,
	mapem_ts_TimeConfidence_time_000_000_2	= 17,
	mapem_ts_TimeConfidence_time_000_000_1	= 18,
	mapem_ts_TimeConfidence_time_000_000_05	= 19,
	mapem_ts_TimeConfidence_time_000_000_02	= 20,
	mapem_ts_TimeConfidence_time_000_000_01	= 21,
	mapem_ts_TimeConfidence_time_000_000_005	= 22,
	mapem_ts_TimeConfidence_time_000_000_002	= 23,
	mapem_ts_TimeConfidence_time_000_000_001	= 24,
	mapem_ts_TimeConfidence_time_000_000_000_5	= 25,
	mapem_ts_TimeConfidence_time_000_000_000_2	= 26,
	mapem_ts_TimeConfidence_time_000_000_000_1	= 27,
	mapem_ts_TimeConfidence_time_000_000_000_05	= 28,
	mapem_ts_TimeConfidence_time_000_000_000_02	= 29,
	mapem_ts_TimeConfidence_time_000_000_000_01	= 30,
	mapem_ts_TimeConfidence_time_000_000_000_005	= 31,
	mapem_ts_TimeConfidence_time_000_000_000_002	= 32,
	mapem_ts_TimeConfidence_time_000_000_000_001	= 33,
	mapem_ts_TimeConfidence_time_000_000_000_000_5	= 34,
	mapem_ts_TimeConfidence_time_000_000_000_000_2	= 35,
	mapem_ts_TimeConfidence_time_000_000_000_000_1	= 36,
	mapem_ts_TimeConfidence_time_000_000_000_000_05	= 37,
	mapem_ts_TimeConfidence_time_000_000_000_000_02	= 38,
	mapem_ts_TimeConfidence_time_000_000_000_000_01	= 39
} e_mapem_ts_TimeConfidence;

/* mapem_ts_TimeConfidence */
typedef long	 mapem_ts_TimeConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mapem_ts_TimeConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_TimeConfidence;
extern const asn_INTEGER_specifics_t asn_SPC_mapem_ts_TimeConfidence_specs_1;
asn_struct_free_f mapem_ts_TimeConfidence_free;
asn_struct_print_f mapem_ts_TimeConfidence_print;
asn_constr_check_f mapem_ts_TimeConfidence_constraint;
per_type_decoder_f mapem_ts_TimeConfidence_decode_uper;
per_type_encoder_f mapem_ts_TimeConfidence_encode_uper;
per_type_decoder_f mapem_ts_TimeConfidence_decode_aper;
per_type_encoder_f mapem_ts_TimeConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_TimeConfidence_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>