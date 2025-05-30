/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_FuelType_H_
#define	_spatem_ts_FuelType_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* spatem_ts_FuelType */
typedef long	 spatem_ts_FuelType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_spatem_ts_FuelType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_FuelType;
asn_struct_free_f spatem_ts_FuelType_free;
asn_struct_print_f spatem_ts_FuelType_print;
asn_constr_check_f spatem_ts_FuelType_constraint;
jer_type_encoder_f spatem_ts_FuelType_encode_jer;
per_type_decoder_f spatem_ts_FuelType_decode_uper;
per_type_encoder_f spatem_ts_FuelType_encode_uper;
per_type_decoder_f spatem_ts_FuelType_decode_aper;
per_type_encoder_f spatem_ts_FuelType_encode_aper;
#define spatem_ts_FuelType_unknownFuel	((spatem_ts_FuelType_t)0)
#define spatem_ts_FuelType_gasoline	((spatem_ts_FuelType_t)1)
#define spatem_ts_FuelType_ethanol	((spatem_ts_FuelType_t)2)
#define spatem_ts_FuelType_diesel	((spatem_ts_FuelType_t)3)
#define spatem_ts_FuelType_electric	((spatem_ts_FuelType_t)4)
#define spatem_ts_FuelType_hybrid	((spatem_ts_FuelType_t)5)
#define spatem_ts_FuelType_hydrogen	((spatem_ts_FuelType_t)6)
#define spatem_ts_FuelType_natGasLiquid	((spatem_ts_FuelType_t)7)
#define spatem_ts_FuelType_natGasComp	((spatem_ts_FuelType_t)8)
#define spatem_ts_FuelType_propane	((spatem_ts_FuelType_t)9)

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_FuelType_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
