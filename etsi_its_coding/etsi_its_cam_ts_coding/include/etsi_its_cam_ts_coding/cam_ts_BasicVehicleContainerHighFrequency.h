/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "/input/CAM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_BasicVehicleContainerHighFrequency_H_
#define	_cam_ts_BasicVehicleContainerHighFrequency_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_ts_coding/cam_ts_Heading.h"
#include "etsi_its_cam_ts_coding/cam_ts_Speed.h"
#include "etsi_its_cam_ts_coding/cam_ts_DriveDirection.h"
#include "etsi_its_cam_ts_coding/cam_ts_VehicleLength.h"
#include "etsi_its_cam_ts_coding/cam_ts_VehicleWidth.h"
#include "etsi_its_cam_ts_coding/cam_ts_AccelerationComponent.h"
#include "etsi_its_cam_ts_coding/cam_ts_Curvature.h"
#include "etsi_its_cam_ts_coding/cam_ts_CurvatureCalculationMode.h"
#include "etsi_its_cam_ts_coding/cam_ts_YawRate.h"
#include "etsi_its_cam_ts_coding/cam_ts_AccelerationControl.h"
#include "etsi_its_cam_ts_coding/cam_ts_LanePosition.h"
#include "etsi_its_cam_ts_coding/cam_ts_PerformanceClass.h"
#include <etsi_its_cam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cam_ts_SteeringWheelAngle;
struct cam_ts_AccelerationComponent;
struct cam_ts_CenDsrcTollingZone;

/* cam_ts_BasicVehicleContainerHighFrequency */
typedef struct cam_ts_BasicVehicleContainerHighFrequency {
	cam_ts_Heading_t	 heading;
	cam_ts_Speed_t	 speed;
	cam_ts_DriveDirection_t	 driveDirection;
	cam_ts_VehicleLength_t	 vehicleLength;
	cam_ts_VehicleWidth_t	 vehicleWidth;
	cam_ts_AccelerationComponent_t	 longitudinalAcceleration;
	cam_ts_Curvature_t	 curvature;
	cam_ts_CurvatureCalculationMode_t	 curvatureCalculationMode;
	cam_ts_YawRate_t	 yawRate;
	cam_ts_AccelerationControl_t	*accelerationControl;	/* OPTIONAL */
	cam_ts_LanePosition_t	*lanePosition;	/* OPTIONAL */
	struct cam_ts_SteeringWheelAngle	*steeringWheelAngle;	/* OPTIONAL */
	struct cam_ts_AccelerationComponent	*lateralAcceleration;	/* OPTIONAL */
	struct cam_ts_AccelerationComponent	*verticalAcceleration;	/* OPTIONAL */
	cam_ts_PerformanceClass_t	*performanceClass;	/* OPTIONAL */
	struct cam_ts_CenDsrcTollingZone	*cenDsrcTollingZone;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_BasicVehicleContainerHighFrequency_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_BasicVehicleContainerHighFrequency;
extern asn_SEQUENCE_specifics_t asn_SPC_cam_ts_BasicVehicleContainerHighFrequency_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_BasicVehicleContainerHighFrequency_1[16];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cam_ts_coding/cam_ts_SteeringWheelAngle.h"
#include "etsi_its_cam_ts_coding/cam_ts_AccelerationComponent.h"
#include "etsi_its_cam_ts_coding/cam_ts_CenDsrcTollingZone.h"

#endif	/* _cam_ts_BasicVehicleContainerHighFrequency_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
