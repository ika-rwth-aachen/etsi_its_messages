/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_ts_VehicleRole_H_
#define	_cpm_ts_VehicleRole_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cpm_ts_VehicleRole {
	cpm_ts_VehicleRole_default	= 0,
	cpm_ts_VehicleRole_publicTransport	= 1,
	cpm_ts_VehicleRole_specialTransport	= 2,
	cpm_ts_VehicleRole_dangerousGoods	= 3,
	cpm_ts_VehicleRole_roadWork	= 4,
	cpm_ts_VehicleRole_rescue	= 5,
	cpm_ts_VehicleRole_emergency	= 6,
	cpm_ts_VehicleRole_safetyCar	= 7,
	cpm_ts_VehicleRole_agriculture	= 8,
	cpm_ts_VehicleRole_commercial	= 9,
	cpm_ts_VehicleRole_military	= 10,
	cpm_ts_VehicleRole_roadOperator	= 11,
	cpm_ts_VehicleRole_taxi	= 12,
	cpm_ts_VehicleRole_reserved1	= 13,
	cpm_ts_VehicleRole_reserved2	= 14,
	cpm_ts_VehicleRole_reserved3	= 15
} e_cpm_ts_VehicleRole;

/* cpm_ts_VehicleRole */
typedef long	 cpm_ts_VehicleRole_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_VehicleRole;
asn_struct_free_f cpm_ts_VehicleRole_free;
asn_struct_print_f cpm_ts_VehicleRole_print;
asn_constr_check_f cpm_ts_VehicleRole_constraint;
per_type_decoder_f cpm_ts_VehicleRole_decode_uper;
per_type_encoder_f cpm_ts_VehicleRole_encode_uper;
per_type_decoder_f cpm_ts_VehicleRole_decode_aper;
per_type_encoder_f cpm_ts_VehicleRole_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_ts_VehicleRole_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>