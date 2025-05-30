/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ElectronicRegistrationIdentificationVehicleDataModule"
 * 	found in "/input/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mapem_ts_EuVehicleCategoryO_H_
#define	_mapem_ts_EuVehicleCategoryO_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_EuVehicleCategoryO {
	mapem_ts_EuVehicleCategoryO_o1	= 0,
	mapem_ts_EuVehicleCategoryO_o2	= 1,
	mapem_ts_EuVehicleCategoryO_o3	= 2,
	mapem_ts_EuVehicleCategoryO_o4	= 3
} e_mapem_ts_EuVehicleCategoryO;

/* mapem_ts_EuVehicleCategoryO */
typedef long	 mapem_ts_EuVehicleCategoryO_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mapem_ts_EuVehicleCategoryO_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_EuVehicleCategoryO;
extern const asn_INTEGER_specifics_t asn_SPC_mapem_ts_EuVehicleCategoryO_specs_1;
asn_struct_free_f mapem_ts_EuVehicleCategoryO_free;
asn_struct_print_f mapem_ts_EuVehicleCategoryO_print;
asn_constr_check_f mapem_ts_EuVehicleCategoryO_constraint;
jer_type_encoder_f mapem_ts_EuVehicleCategoryO_encode_jer;
per_type_decoder_f mapem_ts_EuVehicleCategoryO_decode_uper;
per_type_encoder_f mapem_ts_EuVehicleCategoryO_encode_uper;
per_type_decoder_f mapem_ts_EuVehicleCategoryO_decode_aper;
per_type_encoder_f mapem_ts_EuVehicleCategoryO_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_EuVehicleCategoryO_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
