/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mapem_ts_CauseCodeType_H_
#define	_mapem_ts_CauseCodeType_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_CauseCodeType {
	mapem_ts_CauseCodeType_reserved	= 0,
	mapem_ts_CauseCodeType_trafficCondition	= 1,
	mapem_ts_CauseCodeType_accident	= 2,
	mapem_ts_CauseCodeType_roadworks	= 3,
	mapem_ts_CauseCodeType_impassability	= 5,
	mapem_ts_CauseCodeType_adverseWeatherCondition_Adhesion	= 6,
	mapem_ts_CauseCodeType_aquaplannning	= 7,
	mapem_ts_CauseCodeType_hazardousLocation_SurfaceCondition	= 9,
	mapem_ts_CauseCodeType_hazardousLocation_ObstacleOnTheRoad	= 10,
	mapem_ts_CauseCodeType_hazardousLocation_AnimalOnTheRoad	= 11,
	mapem_ts_CauseCodeType_humanPresenceOnTheRoad	= 12,
	mapem_ts_CauseCodeType_wrongWayDriving	= 14,
	mapem_ts_CauseCodeType_rescueAndRecoveryWorkInProgress	= 15,
	mapem_ts_CauseCodeType_adverseWeatherCondition_ExtremeWeatherCondition	= 17,
	mapem_ts_CauseCodeType_adverseWeatherCondition_Visibility	= 18,
	mapem_ts_CauseCodeType_adverseWeatherCondition_Precipitation	= 19,
	mapem_ts_CauseCodeType_slowVehicle	= 26,
	mapem_ts_CauseCodeType_dangerousEndOfQueue	= 27,
	mapem_ts_CauseCodeType_vehicleBreakdown	= 91,
	mapem_ts_CauseCodeType_postCrash	= 92,
	mapem_ts_CauseCodeType_humanProblem	= 93,
	mapem_ts_CauseCodeType_stationaryVehicle	= 94,
	mapem_ts_CauseCodeType_emergencyVehicleApproaching	= 95,
	mapem_ts_CauseCodeType_hazardousLocation_DangerousCurve	= 96,
	mapem_ts_CauseCodeType_collisionRisk	= 97,
	mapem_ts_CauseCodeType_signalViolation	= 98,
	mapem_ts_CauseCodeType_dangerousSituation	= 99
} e_mapem_ts_CauseCodeType;

/* mapem_ts_CauseCodeType */
typedef long	 mapem_ts_CauseCodeType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mapem_ts_CauseCodeType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_CauseCodeType;
asn_struct_free_f mapem_ts_CauseCodeType_free;
asn_struct_print_f mapem_ts_CauseCodeType_print;
asn_constr_check_f mapem_ts_CauseCodeType_constraint;
jer_type_encoder_f mapem_ts_CauseCodeType_encode_jer;
per_type_decoder_f mapem_ts_CauseCodeType_decode_uper;
per_type_encoder_f mapem_ts_CauseCodeType_encode_uper;
per_type_decoder_f mapem_ts_CauseCodeType_decode_aper;
per_type_encoder_f mapem_ts_CauseCodeType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_CauseCodeType_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
