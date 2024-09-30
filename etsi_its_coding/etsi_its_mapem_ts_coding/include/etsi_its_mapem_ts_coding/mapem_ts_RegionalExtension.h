/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */



/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/mapem_ts_RegionId.h"
#include <etsi_its_mapem_ts_coding/ANY.h>
#include <etsi_its_mapem_ts_coding/asn_ioc.h>
#include "etsi_its_mapem_ts_coding/mapem_ts_MapData-addGrpC.h"
#include <etsi_its_mapem_ts_coding/OPEN_TYPE.h>
#include <etsi_its_mapem_ts_coding/constr_CHOICE.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#include "etsi_its_mapem_ts_coding/mapem_ts_ConnectionManeuverAssist-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_ConnectionTrajectory-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_IntersectionState-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_LaneAttributes-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_MovementEvent-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_NodeAttributeSet-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_Position3D-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_RequestorDescription-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_RestrictionUserType-addGrpC.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_SignalStatusPackage-addGrpC.h"
#ifndef	_mapem_ts_RegionalExtension_H_
#define	_mapem_ts_RegionalExtension_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_Reg_MapData__regExtValue_PR {
	mapem_ts_Reg_MapData__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_MapData__regExtValue_PR_MapData_addGrpC
} mapem_ts_Reg_MapData__regExtValue_PR;
typedef enum mapem_ts_Reg_RTCMcorrections__regExtValue_PR {
	mapem_ts_Reg_RTCMcorrections__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_RTCMcorrections__regExtValue_PR;
typedef enum mapem_ts_Reg_SPAT__regExtValue_PR {
	mapem_ts_Reg_SPAT__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SPAT__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalRequestMessage__regExtValue_PR {
	mapem_ts_Reg_SignalRequestMessage__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SignalRequestMessage__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalStatusMessage__regExtValue_PR {
	mapem_ts_Reg_SignalStatusMessage__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SignalStatusMessage__regExtValue_PR;
typedef enum mapem_ts_Reg_AdvisorySpeed__regExtValue_PR {
	mapem_ts_Reg_AdvisorySpeed__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_AdvisorySpeed__regExtValue_PR;
typedef enum mapem_ts_Reg_ComputedLane__regExtValue_PR {
	mapem_ts_Reg_ComputedLane__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_ComputedLane__regExtValue_PR;
typedef enum mapem_ts_Reg_ConnectionManeuverAssist__regExtValue_PR {
	mapem_ts_Reg_ConnectionManeuverAssist__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_ConnectionManeuverAssist__regExtValue_PR_ConnectionManeuverAssist_addGrpC
} mapem_ts_Reg_ConnectionManeuverAssist__regExtValue_PR;
typedef enum mapem_ts_Reg_GenericLane__regExtValue_PR {
	mapem_ts_Reg_GenericLane__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_GenericLane__regExtValue_PR_ConnectionTrajectory_addGrpC
} mapem_ts_Reg_GenericLane__regExtValue_PR;
typedef enum mapem_ts_Reg_IntersectionGeometry__regExtValue_PR {
	mapem_ts_Reg_IntersectionGeometry__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_IntersectionGeometry__regExtValue_PR;
typedef enum mapem_ts_Reg_IntersectionState__regExtValue_PR {
	mapem_ts_Reg_IntersectionState__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_IntersectionState__regExtValue_PR_IntersectionState_addGrpC
} mapem_ts_Reg_IntersectionState__regExtValue_PR;
typedef enum mapem_ts_Reg_LaneAttributes__regExtValue_PR {
	mapem_ts_Reg_LaneAttributes__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_LaneAttributes__regExtValue_PR_LaneAttributes_addGrpC
} mapem_ts_Reg_LaneAttributes__regExtValue_PR;
typedef enum mapem_ts_Reg_LaneDataAttribute__regExtValue_PR {
	mapem_ts_Reg_LaneDataAttribute__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_LaneDataAttribute__regExtValue_PR;
typedef enum mapem_ts_Reg_MovementEvent__regExtValue_PR {
	mapem_ts_Reg_MovementEvent__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_MovementEvent__regExtValue_PR_MovementEvent_addGrpC
} mapem_ts_Reg_MovementEvent__regExtValue_PR;
typedef enum mapem_ts_Reg_MovementState__regExtValue_PR {
	mapem_ts_Reg_MovementState__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_MovementState__regExtValue_PR;
typedef enum mapem_ts_Reg_NodeAttributeSetXY__regExtValue_PR {
	mapem_ts_Reg_NodeAttributeSetXY__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_NodeAttributeSetXY__regExtValue_PR_NodeAttributeSet_addGrpC
} mapem_ts_Reg_NodeAttributeSetXY__regExtValue_PR;
typedef enum mapem_ts_Reg_NodeOffsetPointXY__regExtValue_PR {
	mapem_ts_Reg_NodeOffsetPointXY__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_NodeOffsetPointXY__regExtValue_PR;
typedef enum mapem_ts_Reg_Position3D__regExtValue_PR {
	mapem_ts_Reg_Position3D__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_Position3D__regExtValue_PR_Position3D_addGrpC
} mapem_ts_Reg_Position3D__regExtValue_PR;
typedef enum mapem_ts_Reg_RequestorDescription__regExtValue_PR {
	mapem_ts_Reg_RequestorDescription__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_RequestorDescription__regExtValue_PR_RequestorDescription_addGrpC
} mapem_ts_Reg_RequestorDescription__regExtValue_PR;
typedef enum mapem_ts_Reg_RequestorType__regExtValue_PR {
	mapem_ts_Reg_RequestorType__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_RequestorType__regExtValue_PR;
typedef enum mapem_ts_Reg_RestrictionUserType__regExtValue_PR {
	mapem_ts_Reg_RestrictionUserType__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_RestrictionUserType__regExtValue_PR_RestrictionUserType_addGrpC
} mapem_ts_Reg_RestrictionUserType__regExtValue_PR;
typedef enum mapem_ts_Reg_RoadSegment__regExtValue_PR {
	mapem_ts_Reg_RoadSegment__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_RoadSegment__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalControlZone__regExtValue_PR {
	mapem_ts_Reg_SignalControlZone__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SignalControlZone__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalRequest__regExtValue_PR {
	mapem_ts_Reg_SignalRequest__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SignalRequest__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalRequestPackage__regExtValue_PR {
	mapem_ts_Reg_SignalRequestPackage__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SignalRequestPackage__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalStatus__regExtValue_PR {
	mapem_ts_Reg_SignalStatus__regExtValue_PR_NOTHING	/* No components present */
	
} mapem_ts_Reg_SignalStatus__regExtValue_PR;
typedef enum mapem_ts_Reg_SignalStatusPackage__regExtValue_PR {
	mapem_ts_Reg_SignalStatusPackage__regExtValue_PR_NOTHING,	/* No components present */
	mapem_ts_Reg_SignalStatusPackage__regExtValue_PR_SignalStatusPackage_addGrpC
} mapem_ts_Reg_SignalStatusPackage__regExtValue_PR;

/* mapem_ts_RegionalExtension */
typedef struct mapem_ts_Reg_MapData {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_MapData__regExtValue {
		mapem_ts_Reg_MapData__regExtValue_PR present;
		union mapem_ts_Reg_MapData__mapem_ts_regExtValue_u {
			mapem_ts_MapData_addGrpC_t	 MapData_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_MapData_t;
typedef struct mapem_ts_Reg_RTCMcorrections {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_RTCMcorrections__regExtValue {
		mapem_ts_Reg_RTCMcorrections__regExtValue_PR present;
		union mapem_ts_Reg_RTCMcorrections__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_RTCMcorrections_t;
typedef struct mapem_ts_Reg_SPAT {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SPAT__regExtValue {
		mapem_ts_Reg_SPAT__regExtValue_PR present;
		union mapem_ts_Reg_SPAT__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SPAT_t;
typedef struct mapem_ts_Reg_SignalRequestMessage {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalRequestMessage__regExtValue {
		mapem_ts_Reg_SignalRequestMessage__regExtValue_PR present;
		union mapem_ts_Reg_SignalRequestMessage__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalRequestMessage_t;
typedef struct mapem_ts_Reg_SignalStatusMessage {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalStatusMessage__regExtValue {
		mapem_ts_Reg_SignalStatusMessage__regExtValue_PR present;
		union mapem_ts_Reg_SignalStatusMessage__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalStatusMessage_t;
typedef struct mapem_ts_Reg_AdvisorySpeed {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_AdvisorySpeed__regExtValue {
		mapem_ts_Reg_AdvisorySpeed__regExtValue_PR present;
		union mapem_ts_Reg_AdvisorySpeed__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_AdvisorySpeed_t;
typedef struct mapem_ts_Reg_ComputedLane {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_ComputedLane__regExtValue {
		mapem_ts_Reg_ComputedLane__regExtValue_PR present;
		union mapem_ts_Reg_ComputedLane__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_ComputedLane_t;
typedef struct mapem_ts_Reg_ConnectionManeuverAssist {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_ConnectionManeuverAssist__regExtValue {
		mapem_ts_Reg_ConnectionManeuverAssist__regExtValue_PR present;
		union mapem_ts_Reg_ConnectionManeuverAssist__mapem_ts_regExtValue_u {
			mapem_ts_ConnectionManeuverAssist_addGrpC_t	 ConnectionManeuverAssist_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_ConnectionManeuverAssist_t;
typedef struct mapem_ts_Reg_GenericLane {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_GenericLane__regExtValue {
		mapem_ts_Reg_GenericLane__regExtValue_PR present;
		union mapem_ts_Reg_GenericLane__mapem_ts_regExtValue_u {
			mapem_ts_ConnectionTrajectory_addGrpC_t	 ConnectionTrajectory_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_GenericLane_t;
typedef struct mapem_ts_Reg_IntersectionGeometry {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_IntersectionGeometry__regExtValue {
		mapem_ts_Reg_IntersectionGeometry__regExtValue_PR present;
		union mapem_ts_Reg_IntersectionGeometry__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_IntersectionGeometry_t;
typedef struct mapem_ts_Reg_IntersectionState {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_IntersectionState__regExtValue {
		mapem_ts_Reg_IntersectionState__regExtValue_PR present;
		union mapem_ts_Reg_IntersectionState__mapem_ts_regExtValue_u {
			mapem_ts_IntersectionState_addGrpC_t	 IntersectionState_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_IntersectionState_t;
typedef struct mapem_ts_Reg_LaneAttributes {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_LaneAttributes__regExtValue {
		mapem_ts_Reg_LaneAttributes__regExtValue_PR present;
		union mapem_ts_Reg_LaneAttributes__mapem_ts_regExtValue_u {
			mapem_ts_LaneAttributes_addGrpC_t	 LaneAttributes_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_LaneAttributes_t;
typedef struct mapem_ts_Reg_LaneDataAttribute {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_LaneDataAttribute__regExtValue {
		mapem_ts_Reg_LaneDataAttribute__regExtValue_PR present;
		union mapem_ts_Reg_LaneDataAttribute__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_LaneDataAttribute_t;
typedef struct mapem_ts_Reg_MovementEvent {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_MovementEvent__regExtValue {
		mapem_ts_Reg_MovementEvent__regExtValue_PR present;
		union mapem_ts_Reg_MovementEvent__mapem_ts_regExtValue_u {
			mapem_ts_MovementEvent_addGrpC_t	 MovementEvent_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_MovementEvent_t;
typedef struct mapem_ts_Reg_MovementState {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_MovementState__regExtValue {
		mapem_ts_Reg_MovementState__regExtValue_PR present;
		union mapem_ts_Reg_MovementState__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_MovementState_t;
typedef struct mapem_ts_Reg_NodeAttributeSetXY {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_NodeAttributeSetXY__regExtValue {
		mapem_ts_Reg_NodeAttributeSetXY__regExtValue_PR present;
		union mapem_ts_Reg_NodeAttributeSetXY__mapem_ts_regExtValue_u {
			mapem_ts_NodeAttributeSet_addGrpC_t	 NodeAttributeSet_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_NodeAttributeSetXY_t;
typedef struct mapem_ts_Reg_NodeOffsetPointXY {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_NodeOffsetPointXY__regExtValue {
		mapem_ts_Reg_NodeOffsetPointXY__regExtValue_PR present;
		union mapem_ts_Reg_NodeOffsetPointXY__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_NodeOffsetPointXY_t;
typedef struct mapem_ts_Reg_Position3D {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_Position3D__regExtValue {
		mapem_ts_Reg_Position3D__regExtValue_PR present;
		union mapem_ts_Reg_Position3D__mapem_ts_regExtValue_u {
			mapem_ts_Position3D_addGrpC_t	 Position3D_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_Position3D_t;
typedef struct mapem_ts_Reg_RequestorDescription {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_RequestorDescription__regExtValue {
		mapem_ts_Reg_RequestorDescription__regExtValue_PR present;
		union mapem_ts_Reg_RequestorDescription__mapem_ts_regExtValue_u {
			mapem_ts_RequestorDescription_addGrpC_t	 RequestorDescription_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_RequestorDescription_t;
typedef struct mapem_ts_Reg_RequestorType {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_RequestorType__regExtValue {
		mapem_ts_Reg_RequestorType__regExtValue_PR present;
		union mapem_ts_Reg_RequestorType__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_RequestorType_t;
typedef struct mapem_ts_Reg_RestrictionUserType {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_RestrictionUserType__regExtValue {
		mapem_ts_Reg_RestrictionUserType__regExtValue_PR present;
		union mapem_ts_Reg_RestrictionUserType__mapem_ts_regExtValue_u {
			mapem_ts_RestrictionUserType_addGrpC_t	 RestrictionUserType_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_RestrictionUserType_t;
typedef struct mapem_ts_Reg_RoadSegment {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_RoadSegment__regExtValue {
		mapem_ts_Reg_RoadSegment__regExtValue_PR present;
		union mapem_ts_Reg_RoadSegment__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_RoadSegment_t;
typedef struct mapem_ts_Reg_SignalControlZone {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalControlZone__regExtValue {
		mapem_ts_Reg_SignalControlZone__regExtValue_PR present;
		union mapem_ts_Reg_SignalControlZone__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalControlZone_t;
typedef struct mapem_ts_Reg_SignalRequest {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalRequest__regExtValue {
		mapem_ts_Reg_SignalRequest__regExtValue_PR present;
		union mapem_ts_Reg_SignalRequest__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalRequest_t;
typedef struct mapem_ts_Reg_SignalRequestPackage {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalRequestPackage__regExtValue {
		mapem_ts_Reg_SignalRequestPackage__regExtValue_PR present;
		union mapem_ts_Reg_SignalRequestPackage__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalRequestPackage_t;
typedef struct mapem_ts_Reg_SignalStatus {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalStatus__regExtValue {
		mapem_ts_Reg_SignalStatus__regExtValue_PR present;
		union mapem_ts_Reg_SignalStatus__mapem_ts_regExtValue_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalStatus_t;
typedef struct mapem_ts_Reg_SignalStatusPackage {
	mapem_ts_RegionId_t	 regionId;
	struct mapem_ts_Reg_SignalStatusPackage__regExtValue {
		mapem_ts_Reg_SignalStatusPackage__regExtValue_PR present;
		union mapem_ts_Reg_SignalStatusPackage__mapem_ts_regExtValue_u {
			mapem_ts_SignalStatusPackage_addGrpC_t	 SignalStatusPackage_addGrpC;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_Reg_SignalStatusPackage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_MapData;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_MapData_specs_1;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_MapData_1[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_RTCMcorrections;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_RTCMcorrections_specs_4;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_RTCMcorrections_4[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SPAT;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SPAT_specs_7;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SPAT_7[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalRequestMessage;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalRequestMessage_specs_10;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalRequestMessage_10[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalStatusMessage;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalStatusMessage_specs_13;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalStatusMessage_13[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_AdvisorySpeed;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_AdvisorySpeed_specs_16;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_AdvisorySpeed_16[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_ComputedLane;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_ComputedLane_specs_19;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_ComputedLane_19[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_ConnectionManeuverAssist;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_ConnectionManeuverAssist_specs_22;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_ConnectionManeuverAssist_22[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_GenericLane;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_GenericLane_specs_25;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_GenericLane_25[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_IntersectionGeometry;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_IntersectionGeometry_specs_28;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_IntersectionGeometry_28[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_IntersectionState;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_IntersectionState_specs_31;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_IntersectionState_31[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_LaneAttributes;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_LaneAttributes_specs_34;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_LaneAttributes_34[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_LaneDataAttribute;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_LaneDataAttribute_specs_37;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_LaneDataAttribute_37[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_MovementEvent;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_MovementEvent_specs_40;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_MovementEvent_40[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_MovementState;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_MovementState_specs_43;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_MovementState_43[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_NodeAttributeSetXY;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_NodeAttributeSetXY_specs_46;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_NodeAttributeSetXY_46[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_NodeOffsetPointXY;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_NodeOffsetPointXY_specs_49;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_NodeOffsetPointXY_49[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_Position3D;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_Position3D_specs_52;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_Position3D_52[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_RequestorDescription;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_RequestorDescription_specs_55;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_RequestorDescription_55[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_RequestorType;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_RequestorType_specs_58;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_RequestorType_58[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_RestrictionUserType;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_RestrictionUserType_specs_61;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_RestrictionUserType_61[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_RoadSegment;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_RoadSegment_specs_64;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_RoadSegment_64[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalControlZone;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalControlZone_specs_67;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalControlZone_67[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalRequest;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalRequest_specs_70;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalRequest_70[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalRequestPackage;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalRequestPackage_specs_73;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalRequestPackage_73[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalStatus;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalStatus_specs_76;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalStatus_76[2];
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_Reg_SignalStatusPackage;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_Reg_SignalStatusPackage_specs_79;
extern asn_TYPE_member_t asn_MBR_mapem_ts_Reg_SignalStatusPackage_79[2];

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_RegionalExtension_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
