/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_CauseCodeType_H_
#define	_cam_CauseCodeType_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_CauseCodeType {
	cam_CauseCodeType_reserved	= 0,
	cam_CauseCodeType_trafficCondition	= 1,
	cam_CauseCodeType_accident	= 2,
	cam_CauseCodeType_roadworks	= 3,
	cam_CauseCodeType_impassability	= 5,
	cam_CauseCodeType_adverseWeatherCondition_Adhesion	= 6,
	cam_CauseCodeType_aquaplannning	= 7,
	cam_CauseCodeType_hazardousLocation_SurfaceCondition	= 9,
	cam_CauseCodeType_hazardousLocation_ObstacleOnTheRoad	= 10,
	cam_CauseCodeType_hazardousLocation_AnimalOnTheRoad	= 11,
	cam_CauseCodeType_humanPresenceOnTheRoad	= 12,
	cam_CauseCodeType_wrongWayDriving	= 14,
	cam_CauseCodeType_rescueAndRecoveryWorkInProgress	= 15,
	cam_CauseCodeType_adverseWeatherCondition_ExtremeWeatherCondition	= 17,
	cam_CauseCodeType_adverseWeatherCondition_Visibility	= 18,
	cam_CauseCodeType_adverseWeatherCondition_Precipitation	= 19,
	cam_CauseCodeType_slowVehicle	= 26,
	cam_CauseCodeType_dangerousEndOfQueue	= 27,
	cam_CauseCodeType_vehicleBreakdown	= 91,
	cam_CauseCodeType_postCrash	= 92,
	cam_CauseCodeType_humanProblem	= 93,
	cam_CauseCodeType_stationaryVehicle	= 94,
	cam_CauseCodeType_emergencyVehicleApproaching	= 95,
	cam_CauseCodeType_hazardousLocation_DangerousCurve	= 96,
	cam_CauseCodeType_collisionRisk	= 97,
	cam_CauseCodeType_signalViolation	= 98,
	cam_CauseCodeType_dangerousSituation	= 99
} e_cam_CauseCodeType;

/* cam_CauseCodeType */
typedef long	 cam_CauseCodeType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_CauseCodeType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_CauseCodeType;
asn_struct_free_f cam_CauseCodeType_free;
asn_struct_print_f cam_CauseCodeType_print;
asn_constr_check_f cam_CauseCodeType_constraint;
jer_type_encoder_f cam_CauseCodeType_encode_jer;
per_type_decoder_f cam_CauseCodeType_decode_uper;
per_type_encoder_f cam_CauseCodeType_encode_uper;
per_type_decoder_f cam_CauseCodeType_decode_aper;
per_type_encoder_f cam_CauseCodeType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_CauseCodeType_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
