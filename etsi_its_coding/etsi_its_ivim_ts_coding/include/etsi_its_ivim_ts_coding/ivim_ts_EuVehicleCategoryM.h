/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ElectronicRegistrationIdentificationVehicleDataModule"
 * 	found in "/input/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_EuVehicleCategoryM_H_
#define	_ivim_ts_EuVehicleCategoryM_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_EuVehicleCategoryM {
	ivim_ts_EuVehicleCategoryM_m1	= 0,
	ivim_ts_EuVehicleCategoryM_m2	= 1,
	ivim_ts_EuVehicleCategoryM_m3	= 2
} e_ivim_ts_EuVehicleCategoryM;

/* ivim_ts_EuVehicleCategoryM */
typedef long	 ivim_ts_EuVehicleCategoryM_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_EuVehicleCategoryM_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_EuVehicleCategoryM;
extern const asn_INTEGER_specifics_t asn_SPC_ivim_ts_EuVehicleCategoryM_specs_1;
asn_struct_free_f ivim_ts_EuVehicleCategoryM_free;
asn_struct_print_f ivim_ts_EuVehicleCategoryM_print;
asn_constr_check_f ivim_ts_EuVehicleCategoryM_constraint;
per_type_decoder_f ivim_ts_EuVehicleCategoryM_decode_uper;
per_type_encoder_f ivim_ts_EuVehicleCategoryM_encode_uper;
per_type_decoder_f ivim_ts_EuVehicleCategoryM_decode_aper;
per_type_encoder_f ivim_ts_EuVehicleCategoryM_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_EuVehicleCategoryM_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
