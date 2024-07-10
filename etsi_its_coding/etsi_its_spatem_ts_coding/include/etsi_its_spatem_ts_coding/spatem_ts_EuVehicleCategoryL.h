/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ElectronicRegistrationIdentificationVehicleDataModule"
 * 	found in "/input/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_spatem_ts_EuVehicleCategoryL_H_
#define	_spatem_ts_EuVehicleCategoryL_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum spatem_ts_EuVehicleCategoryL {
	spatem_ts_EuVehicleCategoryL_l1	= 0,
	spatem_ts_EuVehicleCategoryL_l2	= 1,
	spatem_ts_EuVehicleCategoryL_l3	= 2,
	spatem_ts_EuVehicleCategoryL_l4	= 3,
	spatem_ts_EuVehicleCategoryL_l5	= 4,
	spatem_ts_EuVehicleCategoryL_l6	= 5,
	spatem_ts_EuVehicleCategoryL_l7	= 6
} e_spatem_ts_EuVehicleCategoryL;

/* spatem_ts_EuVehicleCategoryL */
typedef long	 spatem_ts_EuVehicleCategoryL_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_spatem_ts_EuVehicleCategoryL_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_EuVehicleCategoryL;
extern const asn_INTEGER_specifics_t asn_SPC_spatem_ts_EuVehicleCategoryL_specs_1;
asn_struct_free_f spatem_ts_EuVehicleCategoryL_free;
asn_struct_print_f spatem_ts_EuVehicleCategoryL_print;
asn_constr_check_f spatem_ts_EuVehicleCategoryL_constraint;
per_type_decoder_f spatem_ts_EuVehicleCategoryL_decode_uper;
per_type_encoder_f spatem_ts_EuVehicleCategoryL_encode_uper;
per_type_decoder_f spatem_ts_EuVehicleCategoryL_decode_aper;
per_type_encoder_f spatem_ts_EuVehicleCategoryL_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_EuVehicleCategoryL_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>