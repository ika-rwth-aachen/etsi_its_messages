/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_AdverseWeatherCondition_AdhesionSubCauseCode_H_
#define	_cpm_AdverseWeatherCondition_AdhesionSubCauseCode_H_


#include <etsi_its_cpm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cpm_AdverseWeatherCondition_AdhesionSubCauseCode {
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_unavailable	= 0,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_heavyFrostOnRoad	= 1,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_fuelOnRoad	= 2,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_mudOnRoad	= 3,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_snowOnRoad	= 4,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_iceOnRoad	= 5,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_blackIceOnRoad	= 6,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_oilOnRoad	= 7,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_looseChippings	= 8,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_instantBlackIce	= 9,
	cpm_AdverseWeatherCondition_AdhesionSubCauseCode_roadsSalted	= 10
} e_cpm_AdverseWeatherCondition_AdhesionSubCauseCode;

/* cpm_AdverseWeatherCondition-AdhesionSubCauseCode */
typedef long	 cpm_AdverseWeatherCondition_AdhesionSubCauseCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cpm_AdverseWeatherCondition_AdhesionSubCauseCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cpm_AdverseWeatherCondition_AdhesionSubCauseCode;
asn_struct_free_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_free;
asn_struct_print_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_print;
asn_constr_check_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_constraint;
per_type_decoder_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_decode_uper;
per_type_encoder_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_encode_uper;
per_type_decoder_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_decode_aper;
per_type_encoder_f cpm_AdverseWeatherCondition_AdhesionSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_AdverseWeatherCondition_AdhesionSubCauseCode_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
