/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_WheelBaseVehicle_H_
#define	_denm_WheelBaseVehicle_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_WheelBaseVehicle {
	denm_WheelBaseVehicle_tenCentimeters	= 1,
	denm_WheelBaseVehicle_unavailable	= 127
} e_denm_WheelBaseVehicle;

/* denm_WheelBaseVehicle */
typedef long	 denm_WheelBaseVehicle_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_WheelBaseVehicle_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_WheelBaseVehicle;
asn_struct_free_f denm_WheelBaseVehicle_free;
asn_struct_print_f denm_WheelBaseVehicle_print;
asn_constr_check_f denm_WheelBaseVehicle_constraint;
jer_type_encoder_f denm_WheelBaseVehicle_encode_jer;
per_type_decoder_f denm_WheelBaseVehicle_decode_uper;
per_type_encoder_f denm_WheelBaseVehicle_encode_uper;
per_type_decoder_f denm_WheelBaseVehicle_decode_aper;
per_type_encoder_f denm_WheelBaseVehicle_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_WheelBaseVehicle_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
