/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_H_
#define	_cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode {
	cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_unavailable	= 0,
	cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_heavyRain	= 1,
	cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_heavySnowfall	= 2,
	cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_softHail	= 3
} e_cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode;

/* cam_ts_AdverseWeatherCondition-PrecipitationSubCauseCode */
typedef long	 cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode;
asn_struct_free_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_free;
asn_struct_print_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_print;
asn_constr_check_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_constraint;
jer_type_encoder_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_encode_jer;
per_type_decoder_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_decode_uper;
per_type_encoder_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_encode_uper;
per_type_decoder_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_decode_aper;
per_type_encoder_f cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
