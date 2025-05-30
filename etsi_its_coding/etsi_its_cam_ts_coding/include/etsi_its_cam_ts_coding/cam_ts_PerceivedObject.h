/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_PerceivedObject_H_
#define	_cam_ts_PerceivedObject_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_ts_coding/cam_ts_Identifier2B.h"
#include "etsi_its_cam_ts_coding/cam_ts_DeltaTimeMilliSecondSigned.h"
#include "etsi_its_cam_ts_coding/cam_ts_CartesianPosition3dWithConfidence.h"
#include "etsi_its_cam_ts_coding/cam_ts_ObjectPerceptionQuality.h"
#include <etsi_its_cam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cam_ts_Velocity3dWithConfidence;
struct cam_ts_Acceleration3dWithConfidence;
struct cam_ts_EulerAnglesWithConfidence;
struct cam_ts_CartesianAngularVelocityComponent;
struct cam_ts_LowerTriangularPositiveSemidefiniteMatrices;
struct cam_ts_ObjectDimension;
struct cam_ts_SequenceOfIdentifier1B;
struct cam_ts_ObjectClassDescription;
struct cam_ts_MapPosition;

/* cam_ts_PerceivedObject */
typedef struct cam_ts_PerceivedObject {
	cam_ts_Identifier2B_t	*objectId;	/* OPTIONAL */
	cam_ts_DeltaTimeMilliSecondSigned_t	 measurementDeltaTime;
	cam_ts_CartesianPosition3dWithConfidence_t	 position;
	struct cam_ts_Velocity3dWithConfidence	*velocity;	/* OPTIONAL */
	struct cam_ts_Acceleration3dWithConfidence	*acceleration;	/* OPTIONAL */
	struct cam_ts_EulerAnglesWithConfidence	*angles;	/* OPTIONAL */
	struct cam_ts_CartesianAngularVelocityComponent	*zAngularVelocity;	/* OPTIONAL */
	struct cam_ts_LowerTriangularPositiveSemidefiniteMatrices	*lowerTriangularCorrelationMatrices;	/* OPTIONAL */
	struct cam_ts_ObjectDimension	*objectDimensionZ;	/* OPTIONAL */
	struct cam_ts_ObjectDimension	*objectDimensionY;	/* OPTIONAL */
	struct cam_ts_ObjectDimension	*objectDimensionX;	/* OPTIONAL */
	cam_ts_DeltaTimeMilliSecondSigned_t	*objectAge;	/* OPTIONAL */
	cam_ts_ObjectPerceptionQuality_t	*objectPerceptionQuality;	/* OPTIONAL */
	struct cam_ts_SequenceOfIdentifier1B	*sensorIdList;	/* OPTIONAL */
	struct cam_ts_ObjectClassDescription	*classification;	/* OPTIONAL */
	struct cam_ts_MapPosition	*mapPosition;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_PerceivedObject_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_PerceivedObject;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cam_ts_coding/cam_ts_Velocity3dWithConfidence.h"
#include "etsi_its_cam_ts_coding/cam_ts_Acceleration3dWithConfidence.h"
#include "etsi_its_cam_ts_coding/cam_ts_EulerAnglesWithConfidence.h"
#include "etsi_its_cam_ts_coding/cam_ts_CartesianAngularVelocityComponent.h"
#include "etsi_its_cam_ts_coding/cam_ts_LowerTriangularPositiveSemidefiniteMatrices.h"
#include "etsi_its_cam_ts_coding/cam_ts_ObjectDimension.h"
#include "etsi_its_cam_ts_coding/cam_ts_SequenceOfIdentifier1B.h"
#include "etsi_its_cam_ts_coding/cam_ts_ObjectClassDescription.h"
#include "etsi_its_cam_ts_coding/cam_ts_MapPosition.h"

#endif	/* _cam_ts_PerceivedObject_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
