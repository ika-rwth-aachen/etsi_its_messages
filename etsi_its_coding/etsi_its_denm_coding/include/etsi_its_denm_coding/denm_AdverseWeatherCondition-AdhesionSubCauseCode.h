/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_AdverseWeatherCondition_AdhesionSubCauseCode_H_
#define	_denm_AdverseWeatherCondition_AdhesionSubCauseCode_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_AdverseWeatherCondition_AdhesionSubCauseCode {
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_unavailable	= 0,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_heavyFrostOnRoad	= 1,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_fuelOnRoad	= 2,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_mudOnRoad	= 3,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_snowOnRoad	= 4,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_iceOnRoad	= 5,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_blackIceOnRoad	= 6,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_oilOnRoad	= 7,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_looseChippings	= 8,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_instantBlackIce	= 9,
	denm_AdverseWeatherCondition_AdhesionSubCauseCode_roadsSalted	= 10
} e_denm_AdverseWeatherCondition_AdhesionSubCauseCode;

/* denm_AdverseWeatherCondition-AdhesionSubCauseCode */
typedef long	 denm_AdverseWeatherCondition_AdhesionSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_AdverseWeatherCondition_AdhesionSubCauseCode;
asn_struct_free_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_free;
asn_struct_print_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_print;
asn_constr_check_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_constraint;
jer_type_encoder_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_encode_jer;
per_type_decoder_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_decode_uper;
per_type_encoder_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_encode_uper;
per_type_decoder_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_decode_aper;
per_type_encoder_f denm_AdverseWeatherCondition_AdhesionSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_AdverseWeatherCondition_AdhesionSubCauseCode_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
