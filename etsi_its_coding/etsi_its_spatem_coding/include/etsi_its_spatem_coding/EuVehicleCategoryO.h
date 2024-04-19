/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ElectronicRegistrationIdentificationVehicleDataModule"
 * 	found in "/input/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/NativeEnumerated.h>
#ifndef	_EuVehicleCategoryO_H_
#define	_EuVehicleCategoryO_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum EuVehicleCategoryO {
	EuVehicleCategoryO_o1	= 0,
	EuVehicleCategoryO_o2	= 1,
	EuVehicleCategoryO_o3	= 2,
	EuVehicleCategoryO_o4	= 3
} e_EuVehicleCategoryO;

/* EuVehicleCategoryO */
typedef long	 EuVehicleCategoryO_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_EuVehicleCategoryO_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_EuVehicleCategoryO;
extern const asn_INTEGER_specifics_t asn_SPC_EuVehicleCategoryO_specs_1;
asn_struct_free_f EuVehicleCategoryO_free;
asn_struct_print_f EuVehicleCategoryO_print;
asn_constr_check_f EuVehicleCategoryO_constraint;
ber_type_decoder_f EuVehicleCategoryO_decode_ber;
der_type_encoder_f EuVehicleCategoryO_encode_der;
xer_type_decoder_f EuVehicleCategoryO_decode_xer;
xer_type_encoder_f EuVehicleCategoryO_encode_xer;
jer_type_encoder_f EuVehicleCategoryO_encode_jer;
oer_type_decoder_f EuVehicleCategoryO_decode_oer;
oer_type_encoder_f EuVehicleCategoryO_encode_oer;
per_type_decoder_f EuVehicleCategoryO_decode_uper;
per_type_encoder_f EuVehicleCategoryO_encode_uper;
per_type_decoder_f EuVehicleCategoryO_decode_aper;
per_type_encoder_f EuVehicleCategoryO_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _EuVehicleCategoryO_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>