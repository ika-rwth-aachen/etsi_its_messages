/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_vam_ts_EuVehicleCategoryL_H_
#define	_vam_ts_EuVehicleCategoryL_H_


#include <etsi_its_vam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_vam_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum vam_ts_EuVehicleCategoryL {
	vam_ts_EuVehicleCategoryL_l1	= 0,
	vam_ts_EuVehicleCategoryL_l2	= 1,
	vam_ts_EuVehicleCategoryL_l3	= 2,
	vam_ts_EuVehicleCategoryL_l4	= 3,
	vam_ts_EuVehicleCategoryL_l5	= 4,
	vam_ts_EuVehicleCategoryL_l6	= 5,
	vam_ts_EuVehicleCategoryL_l7	= 6
} e_vam_ts_EuVehicleCategoryL;

/* vam_ts_EuVehicleCategoryL */
typedef long	 vam_ts_EuVehicleCategoryL_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_vam_ts_EuVehicleCategoryL_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_vam_ts_EuVehicleCategoryL;
extern const asn_INTEGER_specifics_t asn_SPC_vam_ts_EuVehicleCategoryL_specs_1;
asn_struct_free_f vam_ts_EuVehicleCategoryL_free;
asn_struct_print_f vam_ts_EuVehicleCategoryL_print;
asn_constr_check_f vam_ts_EuVehicleCategoryL_constraint;
per_type_decoder_f vam_ts_EuVehicleCategoryL_decode_uper;
per_type_encoder_f vam_ts_EuVehicleCategoryL_encode_uper;
per_type_decoder_f vam_ts_EuVehicleCategoryL_decode_aper;
per_type_encoder_f vam_ts_EuVehicleCategoryL_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _vam_ts_EuVehicleCategoryL_H_ */
#include <etsi_its_vam_ts_coding/asn_internal.h>