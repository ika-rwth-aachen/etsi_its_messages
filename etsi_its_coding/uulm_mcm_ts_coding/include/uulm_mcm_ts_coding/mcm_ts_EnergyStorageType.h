/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_EnergyStorageType_H_
#define	_mcm_ts_EnergyStorageType_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_EnergyStorageType {
	mcm_ts_EnergyStorageType_hydrogenStorage	= 0,
	mcm_ts_EnergyStorageType_electricEnergyStorage	= 1,
	mcm_ts_EnergyStorageType_liquidPropaneGas	= 2,
	mcm_ts_EnergyStorageType_compressedNaturalGas	= 3,
	mcm_ts_EnergyStorageType_diesel	= 4,
	mcm_ts_EnergyStorageType_gasoline	= 5,
	mcm_ts_EnergyStorageType_ammonia	= 6
} e_mcm_ts_EnergyStorageType;

/* mcm_ts_EnergyStorageType */
typedef BIT_STRING_t	 mcm_ts_EnergyStorageType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_EnergyStorageType;
asn_struct_free_f mcm_ts_EnergyStorageType_free;
asn_struct_print_f mcm_ts_EnergyStorageType_print;
asn_constr_check_f mcm_ts_EnergyStorageType_constraint;
per_type_decoder_f mcm_ts_EnergyStorageType_decode_uper;
per_type_encoder_f mcm_ts_EnergyStorageType_encode_uper;
per_type_decoder_f mcm_ts_EnergyStorageType_decode_aper;
per_type_encoder_f mcm_ts_EnergyStorageType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_EnergyStorageType_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
