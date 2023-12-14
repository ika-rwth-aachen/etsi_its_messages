/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ElectronicRegistrationIdentificationVehicleDataModule"
 * 	found in "/input/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_coding/NativeEnumerated.h>
#ifndef	_EuVehicleCategoryM_H_
#define	_EuVehicleCategoryM_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum EuVehicleCategoryM {
	EuVehicleCategoryM_m1	= 0,
	EuVehicleCategoryM_m2	= 1,
	EuVehicleCategoryM_m3	= 2
} e_EuVehicleCategoryM;

/* EuVehicleCategoryM */
typedef long	 EuVehicleCategoryM_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_EuVehicleCategoryM_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_EuVehicleCategoryM;
extern const asn_INTEGER_specifics_t asn_SPC_EuVehicleCategoryM_specs_1;
asn_struct_free_f EuVehicleCategoryM_free;
asn_struct_print_f EuVehicleCategoryM_print;
asn_constr_check_f EuVehicleCategoryM_constraint;
ber_type_decoder_f EuVehicleCategoryM_decode_ber;
der_type_encoder_f EuVehicleCategoryM_encode_der;
xer_type_decoder_f EuVehicleCategoryM_decode_xer;
xer_type_encoder_f EuVehicleCategoryM_encode_xer;
jer_type_encoder_f EuVehicleCategoryM_encode_jer;
oer_type_decoder_f EuVehicleCategoryM_decode_oer;
oer_type_encoder_f EuVehicleCategoryM_encode_oer;
per_type_decoder_f EuVehicleCategoryM_decode_uper;
per_type_encoder_f EuVehicleCategoryM_encode_uper;
per_type_decoder_f EuVehicleCategoryM_decode_aper;
per_type_encoder_f EuVehicleCategoryM_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _EuVehicleCategoryM_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
