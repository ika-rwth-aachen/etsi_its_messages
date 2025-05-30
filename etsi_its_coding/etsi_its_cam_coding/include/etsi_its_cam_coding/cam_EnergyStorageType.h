/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_EnergyStorageType_H_
#define	_cam_EnergyStorageType_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_EnergyStorageType {
	cam_EnergyStorageType_hydrogenStorage	= 0,
	cam_EnergyStorageType_electricEnergyStorage	= 1,
	cam_EnergyStorageType_liquidPropaneGas	= 2,
	cam_EnergyStorageType_compressedNaturalGas	= 3,
	cam_EnergyStorageType_diesel	= 4,
	cam_EnergyStorageType_gasoline	= 5,
	cam_EnergyStorageType_ammonia	= 6
} e_cam_EnergyStorageType;

/* cam_EnergyStorageType */
typedef BIT_STRING_t	 cam_EnergyStorageType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_EnergyStorageType;
asn_struct_free_f cam_EnergyStorageType_free;
asn_struct_print_f cam_EnergyStorageType_print;
asn_constr_check_f cam_EnergyStorageType_constraint;
jer_type_encoder_f cam_EnergyStorageType_encode_jer;
per_type_decoder_f cam_EnergyStorageType_decode_uper;
per_type_encoder_f cam_EnergyStorageType_encode_uper;
per_type_decoder_f cam_EnergyStorageType_decode_aper;
per_type_encoder_f cam_EnergyStorageType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_EnergyStorageType_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
