/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cpm_ts_EuVehicleCategoryO_H_
#define	_cpm_ts_EuVehicleCategoryO_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cpm_ts_EuVehicleCategoryO {
	cpm_ts_EuVehicleCategoryO_o1	= 0,
	cpm_ts_EuVehicleCategoryO_o2	= 1,
	cpm_ts_EuVehicleCategoryO_o3	= 2,
	cpm_ts_EuVehicleCategoryO_o4	= 3
} e_cpm_ts_EuVehicleCategoryO;

/* cpm_ts_EuVehicleCategoryO */
typedef long	 cpm_ts_EuVehicleCategoryO_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cpm_ts_EuVehicleCategoryO_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_EuVehicleCategoryO;
extern const asn_INTEGER_specifics_t asn_SPC_cpm_ts_EuVehicleCategoryO_specs_1;
asn_struct_free_f cpm_ts_EuVehicleCategoryO_free;
asn_struct_print_f cpm_ts_EuVehicleCategoryO_print;
asn_constr_check_f cpm_ts_EuVehicleCategoryO_constraint;
jer_type_encoder_f cpm_ts_EuVehicleCategoryO_encode_jer;
per_type_decoder_f cpm_ts_EuVehicleCategoryO_decode_uper;
per_type_encoder_f cpm_ts_EuVehicleCategoryO_encode_uper;
per_type_decoder_f cpm_ts_EuVehicleCategoryO_decode_aper;
per_type_encoder_f cpm_ts_EuVehicleCategoryO_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_ts_EuVehicleCategoryO_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>
