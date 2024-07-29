/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cam_ts_CauseCodeChoice_H_
#define	_cam_ts_CauseCodeChoice_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_ts_coding/cam_ts_SubCauseCodeType.h"
#include "etsi_its_cam_ts_coding/cam_ts_TrafficConditionSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_AccidentSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_RoadworksSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_ImpassabilitySubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_AdverseWeatherCondition-AdhesionSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_HazardousLocation-SurfaceConditionSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_HazardousLocation-ObstacleOnTheRoadSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_HazardousLocation-AnimalOnTheRoadSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_HumanPresenceOnTheRoadSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_WrongWayDrivingSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_RescueAndRecoveryWorkInProgressSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_AdverseWeatherCondition-VisibilitySubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_AdverseWeatherCondition-PrecipitationSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_SlowVehicleSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_DangerousEndOfQueueSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_VehicleBreakdownSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_PostCrashSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_HumanProblemSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_StationaryVehicleSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_EmergencyVehicleApproachingSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_HazardousLocation-DangerousCurveSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_CollisionRiskSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_SignalViolationSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_DangerousSituationSubCauseCode.h"
#include "etsi_its_cam_ts_coding/cam_ts_RailwayLevelCrossingSubCauseCode.h"
#include <etsi_its_cam_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_CauseCodeChoice_PR {
	cam_ts_CauseCodeChoice_PR_NOTHING,	/* No components present */
	cam_ts_CauseCodeChoice_PR_reserved0,
	cam_ts_CauseCodeChoice_PR_trafficCondition1,
	cam_ts_CauseCodeChoice_PR_accident2,
	cam_ts_CauseCodeChoice_PR_roadworks3,
	cam_ts_CauseCodeChoice_PR_reserved4,
	cam_ts_CauseCodeChoice_PR_impassability5,
	cam_ts_CauseCodeChoice_PR_adverseWeatherCondition_Adhesion6,
	cam_ts_CauseCodeChoice_PR_aquaplaning7,
	cam_ts_CauseCodeChoice_PR_reserved8,
	cam_ts_CauseCodeChoice_PR_hazardousLocation_SurfaceCondition9,
	cam_ts_CauseCodeChoice_PR_hazardousLocation_ObstacleOnTheRoad10,
	cam_ts_CauseCodeChoice_PR_hazardousLocation_AnimalOnTheRoad11,
	cam_ts_CauseCodeChoice_PR_humanPresenceOnTheRoad12,
	cam_ts_CauseCodeChoice_PR_reserved13,
	cam_ts_CauseCodeChoice_PR_wrongWayDriving14,
	cam_ts_CauseCodeChoice_PR_rescueAndRecoveryWorkInProgress15,
	cam_ts_CauseCodeChoice_PR_reserved16,
	cam_ts_CauseCodeChoice_PR_adverseWeatherCondition_ExtremeWeatherCondition17,
	cam_ts_CauseCodeChoice_PR_adverseWeatherCondition_Visibility18,
	cam_ts_CauseCodeChoice_PR_adverseWeatherCondition_Precipitation19,
	cam_ts_CauseCodeChoice_PR_violence20,
	cam_ts_CauseCodeChoice_PR_reserved21,
	cam_ts_CauseCodeChoice_PR_reserved22,
	cam_ts_CauseCodeChoice_PR_reserved23,
	cam_ts_CauseCodeChoice_PR_reserved24,
	cam_ts_CauseCodeChoice_PR_reserved25,
	cam_ts_CauseCodeChoice_PR_slowVehicle26,
	cam_ts_CauseCodeChoice_PR_dangerousEndOfQueue27,
	cam_ts_CauseCodeChoice_PR_publicTransportVehicleApproaching28,
	cam_ts_CauseCodeChoice_PR_reserved29,
	cam_ts_CauseCodeChoice_PR_reserved30,
	cam_ts_CauseCodeChoice_PR_reserved31,
	cam_ts_CauseCodeChoice_PR_reserved32,
	cam_ts_CauseCodeChoice_PR_reserved33,
	cam_ts_CauseCodeChoice_PR_reserved34,
	cam_ts_CauseCodeChoice_PR_reserved35,
	cam_ts_CauseCodeChoice_PR_reserved36,
	cam_ts_CauseCodeChoice_PR_reserved37,
	cam_ts_CauseCodeChoice_PR_reserved38,
	cam_ts_CauseCodeChoice_PR_reserved39,
	cam_ts_CauseCodeChoice_PR_reserved40,
	cam_ts_CauseCodeChoice_PR_reserved41,
	cam_ts_CauseCodeChoice_PR_reserved42,
	cam_ts_CauseCodeChoice_PR_reserved43,
	cam_ts_CauseCodeChoice_PR_reserved44,
	cam_ts_CauseCodeChoice_PR_reserved45,
	cam_ts_CauseCodeChoice_PR_reserved46,
	cam_ts_CauseCodeChoice_PR_reserved47,
	cam_ts_CauseCodeChoice_PR_reserved48,
	cam_ts_CauseCodeChoice_PR_reserved49,
	cam_ts_CauseCodeChoice_PR_reserved50,
	cam_ts_CauseCodeChoice_PR_reserved51,
	cam_ts_CauseCodeChoice_PR_reserved52,
	cam_ts_CauseCodeChoice_PR_reserved53,
	cam_ts_CauseCodeChoice_PR_reserved54,
	cam_ts_CauseCodeChoice_PR_reserved55,
	cam_ts_CauseCodeChoice_PR_reserved56,
	cam_ts_CauseCodeChoice_PR_reserved57,
	cam_ts_CauseCodeChoice_PR_reserved58,
	cam_ts_CauseCodeChoice_PR_reserved59,
	cam_ts_CauseCodeChoice_PR_reserved60,
	cam_ts_CauseCodeChoice_PR_reserved61,
	cam_ts_CauseCodeChoice_PR_reserved62,
	cam_ts_CauseCodeChoice_PR_reserved63,
	cam_ts_CauseCodeChoice_PR_reserved64,
	cam_ts_CauseCodeChoice_PR_reserved65,
	cam_ts_CauseCodeChoice_PR_reserved66,
	cam_ts_CauseCodeChoice_PR_reserved67,
	cam_ts_CauseCodeChoice_PR_reserved68,
	cam_ts_CauseCodeChoice_PR_reserved69,
	cam_ts_CauseCodeChoice_PR_reserved70,
	cam_ts_CauseCodeChoice_PR_reserved71,
	cam_ts_CauseCodeChoice_PR_reserved72,
	cam_ts_CauseCodeChoice_PR_reserved73,
	cam_ts_CauseCodeChoice_PR_reserved74,
	cam_ts_CauseCodeChoice_PR_reserved75,
	cam_ts_CauseCodeChoice_PR_reserved76,
	cam_ts_CauseCodeChoice_PR_reserved77,
	cam_ts_CauseCodeChoice_PR_reserved78,
	cam_ts_CauseCodeChoice_PR_reserved79,
	cam_ts_CauseCodeChoice_PR_reserved80,
	cam_ts_CauseCodeChoice_PR_reserved81,
	cam_ts_CauseCodeChoice_PR_reserved82,
	cam_ts_CauseCodeChoice_PR_reserved83,
	cam_ts_CauseCodeChoice_PR_reserved84,
	cam_ts_CauseCodeChoice_PR_reserved85,
	cam_ts_CauseCodeChoice_PR_reserved86,
	cam_ts_CauseCodeChoice_PR_reserved87,
	cam_ts_CauseCodeChoice_PR_reserved88,
	cam_ts_CauseCodeChoice_PR_reserved89,
	cam_ts_CauseCodeChoice_PR_reserved90,
	cam_ts_CauseCodeChoice_PR_vehicleBreakdown91,
	cam_ts_CauseCodeChoice_PR_postCrash92,
	cam_ts_CauseCodeChoice_PR_humanProblem93,
	cam_ts_CauseCodeChoice_PR_stationaryVehicle94,
	cam_ts_CauseCodeChoice_PR_emergencyVehicleApproaching95,
	cam_ts_CauseCodeChoice_PR_hazardousLocation_DangerousCurve96,
	cam_ts_CauseCodeChoice_PR_collisionRisk97,
	cam_ts_CauseCodeChoice_PR_signalViolation98,
	cam_ts_CauseCodeChoice_PR_dangerousSituation99,
	cam_ts_CauseCodeChoice_PR_railwayLevelCrossing100,
	cam_ts_CauseCodeChoice_PR_reserved101,
	cam_ts_CauseCodeChoice_PR_reserved102,
	cam_ts_CauseCodeChoice_PR_reserved103,
	cam_ts_CauseCodeChoice_PR_reserved104,
	cam_ts_CauseCodeChoice_PR_reserved105,
	cam_ts_CauseCodeChoice_PR_reserved106,
	cam_ts_CauseCodeChoice_PR_reserved107,
	cam_ts_CauseCodeChoice_PR_reserved108,
	cam_ts_CauseCodeChoice_PR_reserved109,
	cam_ts_CauseCodeChoice_PR_reserved110,
	cam_ts_CauseCodeChoice_PR_reserved111,
	cam_ts_CauseCodeChoice_PR_reserved112,
	cam_ts_CauseCodeChoice_PR_reserved113,
	cam_ts_CauseCodeChoice_PR_reserved114,
	cam_ts_CauseCodeChoice_PR_reserved115,
	cam_ts_CauseCodeChoice_PR_reserved116,
	cam_ts_CauseCodeChoice_PR_reserved117,
	cam_ts_CauseCodeChoice_PR_reserved118,
	cam_ts_CauseCodeChoice_PR_reserved119,
	cam_ts_CauseCodeChoice_PR_reserved120,
	cam_ts_CauseCodeChoice_PR_reserved121,
	cam_ts_CauseCodeChoice_PR_reserved122,
	cam_ts_CauseCodeChoice_PR_reserved123,
	cam_ts_CauseCodeChoice_PR_reserved124,
	cam_ts_CauseCodeChoice_PR_reserved125,
	cam_ts_CauseCodeChoice_PR_reserved126,
	cam_ts_CauseCodeChoice_PR_reserved127,
	cam_ts_CauseCodeChoice_PR_reserved128
} cam_ts_CauseCodeChoice_PR;

/* cam_ts_CauseCodeChoice */
typedef struct cam_ts_CauseCodeChoice {
	cam_ts_CauseCodeChoice_PR present;
	union cam_ts_CauseCodeChoice_u {
		cam_ts_SubCauseCodeType_t	 reserved0;
		cam_ts_TrafficConditionSubCauseCode_t	 trafficCondition1;
		cam_ts_AccidentSubCauseCode_t	 accident2;
		cam_ts_RoadworksSubCauseCode_t	 roadworks3;
		cam_ts_SubCauseCodeType_t	 reserved4;
		cam_ts_ImpassabilitySubCauseCode_t	 impassability5;
		cam_ts_AdverseWeatherCondition_AdhesionSubCauseCode_t	 adverseWeatherCondition_Adhesion6;
		cam_ts_SubCauseCodeType_t	 aquaplaning7;
		cam_ts_SubCauseCodeType_t	 reserved8;
		cam_ts_HazardousLocation_SurfaceConditionSubCauseCode_t	 hazardousLocation_SurfaceCondition9;
		cam_ts_HazardousLocation_ObstacleOnTheRoadSubCauseCode_t	 hazardousLocation_ObstacleOnTheRoad10;
		cam_ts_HazardousLocation_AnimalOnTheRoadSubCauseCode_t	 hazardousLocation_AnimalOnTheRoad11;
		cam_ts_HumanPresenceOnTheRoadSubCauseCode_t	 humanPresenceOnTheRoad12;
		cam_ts_SubCauseCodeType_t	 reserved13;
		cam_ts_WrongWayDrivingSubCauseCode_t	 wrongWayDriving14;
		cam_ts_RescueAndRecoveryWorkInProgressSubCauseCode_t	 rescueAndRecoveryWorkInProgress15;
		cam_ts_SubCauseCodeType_t	 reserved16;
		cam_ts_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_t	 adverseWeatherCondition_ExtremeWeatherCondition17;
		cam_ts_AdverseWeatherCondition_VisibilitySubCauseCode_t	 adverseWeatherCondition_Visibility18;
		cam_ts_AdverseWeatherCondition_PrecipitationSubCauseCode_t	 adverseWeatherCondition_Precipitation19;
		cam_ts_SubCauseCodeType_t	 violence20;
		cam_ts_SubCauseCodeType_t	 reserved21;
		cam_ts_SubCauseCodeType_t	 reserved22;
		cam_ts_SubCauseCodeType_t	 reserved23;
		cam_ts_SubCauseCodeType_t	 reserved24;
		cam_ts_SubCauseCodeType_t	 reserved25;
		cam_ts_SlowVehicleSubCauseCode_t	 slowVehicle26;
		cam_ts_DangerousEndOfQueueSubCauseCode_t	 dangerousEndOfQueue27;
		cam_ts_SubCauseCodeType_t	 publicTransportVehicleApproaching28;
		cam_ts_SubCauseCodeType_t	 reserved29;
		cam_ts_SubCauseCodeType_t	 reserved30;
		cam_ts_SubCauseCodeType_t	 reserved31;
		cam_ts_SubCauseCodeType_t	 reserved32;
		cam_ts_SubCauseCodeType_t	 reserved33;
		cam_ts_SubCauseCodeType_t	 reserved34;
		cam_ts_SubCauseCodeType_t	 reserved35;
		cam_ts_SubCauseCodeType_t	 reserved36;
		cam_ts_SubCauseCodeType_t	 reserved37;
		cam_ts_SubCauseCodeType_t	 reserved38;
		cam_ts_SubCauseCodeType_t	 reserved39;
		cam_ts_SubCauseCodeType_t	 reserved40;
		cam_ts_SubCauseCodeType_t	 reserved41;
		cam_ts_SubCauseCodeType_t	 reserved42;
		cam_ts_SubCauseCodeType_t	 reserved43;
		cam_ts_SubCauseCodeType_t	 reserved44;
		cam_ts_SubCauseCodeType_t	 reserved45;
		cam_ts_SubCauseCodeType_t	 reserved46;
		cam_ts_SubCauseCodeType_t	 reserved47;
		cam_ts_SubCauseCodeType_t	 reserved48;
		cam_ts_SubCauseCodeType_t	 reserved49;
		cam_ts_SubCauseCodeType_t	 reserved50;
		cam_ts_SubCauseCodeType_t	 reserved51;
		cam_ts_SubCauseCodeType_t	 reserved52;
		cam_ts_SubCauseCodeType_t	 reserved53;
		cam_ts_SubCauseCodeType_t	 reserved54;
		cam_ts_SubCauseCodeType_t	 reserved55;
		cam_ts_SubCauseCodeType_t	 reserved56;
		cam_ts_SubCauseCodeType_t	 reserved57;
		cam_ts_SubCauseCodeType_t	 reserved58;
		cam_ts_SubCauseCodeType_t	 reserved59;
		cam_ts_SubCauseCodeType_t	 reserved60;
		cam_ts_SubCauseCodeType_t	 reserved61;
		cam_ts_SubCauseCodeType_t	 reserved62;
		cam_ts_SubCauseCodeType_t	 reserved63;
		cam_ts_SubCauseCodeType_t	 reserved64;
		cam_ts_SubCauseCodeType_t	 reserved65;
		cam_ts_SubCauseCodeType_t	 reserved66;
		cam_ts_SubCauseCodeType_t	 reserved67;
		cam_ts_SubCauseCodeType_t	 reserved68;
		cam_ts_SubCauseCodeType_t	 reserved69;
		cam_ts_SubCauseCodeType_t	 reserved70;
		cam_ts_SubCauseCodeType_t	 reserved71;
		cam_ts_SubCauseCodeType_t	 reserved72;
		cam_ts_SubCauseCodeType_t	 reserved73;
		cam_ts_SubCauseCodeType_t	 reserved74;
		cam_ts_SubCauseCodeType_t	 reserved75;
		cam_ts_SubCauseCodeType_t	 reserved76;
		cam_ts_SubCauseCodeType_t	 reserved77;
		cam_ts_SubCauseCodeType_t	 reserved78;
		cam_ts_SubCauseCodeType_t	 reserved79;
		cam_ts_SubCauseCodeType_t	 reserved80;
		cam_ts_SubCauseCodeType_t	 reserved81;
		cam_ts_SubCauseCodeType_t	 reserved82;
		cam_ts_SubCauseCodeType_t	 reserved83;
		cam_ts_SubCauseCodeType_t	 reserved84;
		cam_ts_SubCauseCodeType_t	 reserved85;
		cam_ts_SubCauseCodeType_t	 reserved86;
		cam_ts_SubCauseCodeType_t	 reserved87;
		cam_ts_SubCauseCodeType_t	 reserved88;
		cam_ts_SubCauseCodeType_t	 reserved89;
		cam_ts_SubCauseCodeType_t	 reserved90;
		cam_ts_VehicleBreakdownSubCauseCode_t	 vehicleBreakdown91;
		cam_ts_PostCrashSubCauseCode_t	 postCrash92;
		cam_ts_HumanProblemSubCauseCode_t	 humanProblem93;
		cam_ts_StationaryVehicleSubCauseCode_t	 stationaryVehicle94;
		cam_ts_EmergencyVehicleApproachingSubCauseCode_t	 emergencyVehicleApproaching95;
		cam_ts_HazardousLocation_DangerousCurveSubCauseCode_t	 hazardousLocation_DangerousCurve96;
		cam_ts_CollisionRiskSubCauseCode_t	 collisionRisk97;
		cam_ts_SignalViolationSubCauseCode_t	 signalViolation98;
		cam_ts_DangerousSituationSubCauseCode_t	 dangerousSituation99;
		cam_ts_RailwayLevelCrossingSubCauseCode_t	 railwayLevelCrossing100;
		cam_ts_SubCauseCodeType_t	 reserved101;
		cam_ts_SubCauseCodeType_t	 reserved102;
		cam_ts_SubCauseCodeType_t	 reserved103;
		cam_ts_SubCauseCodeType_t	 reserved104;
		cam_ts_SubCauseCodeType_t	 reserved105;
		cam_ts_SubCauseCodeType_t	 reserved106;
		cam_ts_SubCauseCodeType_t	 reserved107;
		cam_ts_SubCauseCodeType_t	 reserved108;
		cam_ts_SubCauseCodeType_t	 reserved109;
		cam_ts_SubCauseCodeType_t	 reserved110;
		cam_ts_SubCauseCodeType_t	 reserved111;
		cam_ts_SubCauseCodeType_t	 reserved112;
		cam_ts_SubCauseCodeType_t	 reserved113;
		cam_ts_SubCauseCodeType_t	 reserved114;
		cam_ts_SubCauseCodeType_t	 reserved115;
		cam_ts_SubCauseCodeType_t	 reserved116;
		cam_ts_SubCauseCodeType_t	 reserved117;
		cam_ts_SubCauseCodeType_t	 reserved118;
		cam_ts_SubCauseCodeType_t	 reserved119;
		cam_ts_SubCauseCodeType_t	 reserved120;
		cam_ts_SubCauseCodeType_t	 reserved121;
		cam_ts_SubCauseCodeType_t	 reserved122;
		cam_ts_SubCauseCodeType_t	 reserved123;
		cam_ts_SubCauseCodeType_t	 reserved124;
		cam_ts_SubCauseCodeType_t	 reserved125;
		cam_ts_SubCauseCodeType_t	 reserved126;
		cam_ts_SubCauseCodeType_t	 reserved127;
		cam_ts_SubCauseCodeType_t	 reserved128;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_CauseCodeChoice_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_CauseCodeChoice;
extern asn_CHOICE_specifics_t asn_SPC_cam_ts_CauseCodeChoice_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_CauseCodeChoice_1[129];
extern asn_per_constraints_t asn_PER_type_cam_ts_CauseCodeChoice_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_CauseCodeChoice_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>