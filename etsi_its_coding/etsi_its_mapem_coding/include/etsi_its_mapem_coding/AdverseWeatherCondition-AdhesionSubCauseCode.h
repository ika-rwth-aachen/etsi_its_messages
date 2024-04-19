/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_coding/NativeInteger.h>
#ifndef	_AdverseWeatherCondition_AdhesionSubCauseCode_H_
#define	_AdverseWeatherCondition_AdhesionSubCauseCode_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AdverseWeatherCondition_AdhesionSubCauseCode {
	AdverseWeatherCondition_AdhesionSubCauseCode_unavailable	= 0,
	AdverseWeatherCondition_AdhesionSubCauseCode_heavyFrostOnRoad	= 1,
	AdverseWeatherCondition_AdhesionSubCauseCode_fuelOnRoad	= 2,
	AdverseWeatherCondition_AdhesionSubCauseCode_mudOnRoad	= 3,
	AdverseWeatherCondition_AdhesionSubCauseCode_snowOnRoad	= 4,
	AdverseWeatherCondition_AdhesionSubCauseCode_iceOnRoad	= 5,
	AdverseWeatherCondition_AdhesionSubCauseCode_blackIceOnRoad	= 6,
	AdverseWeatherCondition_AdhesionSubCauseCode_oilOnRoad	= 7,
	AdverseWeatherCondition_AdhesionSubCauseCode_looseChippings	= 8,
	AdverseWeatherCondition_AdhesionSubCauseCode_instantBlackIce	= 9,
	AdverseWeatherCondition_AdhesionSubCauseCode_roadsSalted	= 10
} e_AdverseWeatherCondition_AdhesionSubCauseCode;

/* AdverseWeatherCondition-AdhesionSubCauseCode */
typedef long	 AdverseWeatherCondition_AdhesionSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AdverseWeatherCondition_AdhesionSubCauseCode;
asn_struct_free_f AdverseWeatherCondition_AdhesionSubCauseCode_free;
asn_struct_print_f AdverseWeatherCondition_AdhesionSubCauseCode_print;
asn_constr_check_f AdverseWeatherCondition_AdhesionSubCauseCode_constraint;
ber_type_decoder_f AdverseWeatherCondition_AdhesionSubCauseCode_decode_ber;
der_type_encoder_f AdverseWeatherCondition_AdhesionSubCauseCode_encode_der;
xer_type_decoder_f AdverseWeatherCondition_AdhesionSubCauseCode_decode_xer;
xer_type_encoder_f AdverseWeatherCondition_AdhesionSubCauseCode_encode_xer;
jer_type_encoder_f AdverseWeatherCondition_AdhesionSubCauseCode_encode_jer;
oer_type_decoder_f AdverseWeatherCondition_AdhesionSubCauseCode_decode_oer;
oer_type_encoder_f AdverseWeatherCondition_AdhesionSubCauseCode_encode_oer;
per_type_decoder_f AdverseWeatherCondition_AdhesionSubCauseCode_decode_uper;
per_type_encoder_f AdverseWeatherCondition_AdhesionSubCauseCode_encode_uper;
per_type_decoder_f AdverseWeatherCondition_AdhesionSubCauseCode_decode_aper;
per_type_encoder_f AdverseWeatherCondition_AdhesionSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _AdverseWeatherCondition_AdhesionSubCauseCode_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>