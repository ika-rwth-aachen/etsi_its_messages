/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_RadialShape_H_
#define	_cam_ts_RadialShape_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_ts_coding/cam_ts_StandardLength12b.h"
#include "etsi_its_cam_ts_coding/cam_ts_CartesianAngleValue.h"
#include <etsi_its_cam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cam_ts_CartesianPosition3d;

/* cam_ts_RadialShape */
typedef struct cam_ts_RadialShape {
	struct cam_ts_CartesianPosition3d	*shapeReferencePoint;	/* OPTIONAL */
	cam_ts_StandardLength12b_t	 range;
	cam_ts_CartesianAngleValue_t	 horizontalOpeningAngleStart;
	cam_ts_CartesianAngleValue_t	 horizontalOpeningAngleEnd;
	cam_ts_CartesianAngleValue_t	*verticalOpeningAngleStart;	/* OPTIONAL */
	cam_ts_CartesianAngleValue_t	*verticalOpeningAngleEnd;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_RadialShape_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_RadialShape;
extern asn_SEQUENCE_specifics_t asn_SPC_cam_ts_RadialShape_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_RadialShape_1[6];
extern asn_per_constraints_t asn_PER_type_cam_ts_RadialShape_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cam_ts_coding/cam_ts_CartesianPosition3d.h"

#endif	/* _cam_ts_RadialShape_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
