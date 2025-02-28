/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_VehicleRole_H_
#define	_mcm_uulm_VehicleRole_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_VehicleRole {
	mcm_uulm_VehicleRole_default	= 0,
	mcm_uulm_VehicleRole_publicTransport	= 1,
	mcm_uulm_VehicleRole_specialTransport	= 2,
	mcm_uulm_VehicleRole_dangerousGoods	= 3,
	mcm_uulm_VehicleRole_roadWork	= 4,
	mcm_uulm_VehicleRole_rescue	= 5,
	mcm_uulm_VehicleRole_emergency	= 6,
	mcm_uulm_VehicleRole_safetyCar	= 7,
	mcm_uulm_VehicleRole_agriculture	= 8,
	mcm_uulm_VehicleRole_commercial	= 9,
	mcm_uulm_VehicleRole_military	= 10,
	mcm_uulm_VehicleRole_roadOperator	= 11,
	mcm_uulm_VehicleRole_taxi	= 12,
	mcm_uulm_VehicleRole_uvar	= 13,
	mcm_uulm_VehicleRole_rfu1	= 14,
	mcm_uulm_VehicleRole_rfu2	= 15
} e_mcm_uulm_VehicleRole;

/* mcm_uulm_VehicleRole */
typedef long	 mcm_uulm_VehicleRole_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_VehicleRole;
asn_struct_free_f mcm_uulm_VehicleRole_free;
asn_struct_print_f mcm_uulm_VehicleRole_print;
asn_constr_check_f mcm_uulm_VehicleRole_constraint;
per_type_decoder_f mcm_uulm_VehicleRole_decode_uper;
per_type_encoder_f mcm_uulm_VehicleRole_encode_uper;
per_type_decoder_f mcm_uulm_VehicleRole_decode_aper;
per_type_encoder_f mcm_uulm_VehicleRole_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_VehicleRole_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
