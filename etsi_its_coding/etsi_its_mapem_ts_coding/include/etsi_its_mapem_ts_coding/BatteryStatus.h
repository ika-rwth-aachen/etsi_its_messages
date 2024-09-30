/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/NativeEnumerated.h>
#ifndef	_BatteryStatus_H_
#define	_BatteryStatus_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BatteryStatus {
	BatteryStatus_unknown	= 0,
	BatteryStatus_critical	= 1,
	BatteryStatus_low	= 2,
	BatteryStatus_good	= 3
	/*
	 * Enumeration is extensible
	 */
} e_BatteryStatus;

/* BatteryStatus */
typedef long	 BatteryStatus_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_BatteryStatus_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_BatteryStatus;
extern const asn_INTEGER_specifics_t asn_SPC_BatteryStatus_specs_1;
asn_struct_free_f BatteryStatus_free;
asn_struct_print_f BatteryStatus_print;
asn_constr_check_f BatteryStatus_constraint;
ber_type_decoder_f BatteryStatus_decode_ber;
der_type_encoder_f BatteryStatus_encode_der;
xer_type_decoder_f BatteryStatus_decode_xer;
xer_type_encoder_f BatteryStatus_encode_xer;
jer_type_encoder_f BatteryStatus_encode_jer;
oer_type_decoder_f BatteryStatus_decode_oer;
oer_type_encoder_f BatteryStatus_encode_oer;
per_type_decoder_f BatteryStatus_decode_uper;
per_type_encoder_f BatteryStatus_encode_uper;
per_type_decoder_f BatteryStatus_decode_aper;
per_type_encoder_f BatteryStatus_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _BatteryStatus_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
