/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_Shape_H_
#define	_cam_ts_Shape_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_ts_coding/cam_ts_RectangularShape.h"
#include "etsi_its_cam_ts_coding/cam_ts_CircularShape.h"
#include "etsi_its_cam_ts_coding/cam_ts_PolygonalShape.h"
#include "etsi_its_cam_ts_coding/cam_ts_EllipticalShape.h"
#include "etsi_its_cam_ts_coding/cam_ts_RadialShape.h"
#include "etsi_its_cam_ts_coding/cam_ts_RadialShapes.h"
#include <etsi_its_cam_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_Shape_PR {
	cam_ts_Shape_PR_NOTHING,	/* No components present */
	cam_ts_Shape_PR_rectangular,
	cam_ts_Shape_PR_circular,
	cam_ts_Shape_PR_polygonal,
	cam_ts_Shape_PR_elliptical,
	cam_ts_Shape_PR_radial,
	cam_ts_Shape_PR_radialShapes
	/* Extensions may appear below */
	
} cam_ts_Shape_PR;

/* cam_ts_Shape */
typedef struct cam_ts_Shape {
	cam_ts_Shape_PR present;
	union cam_ts_Shape_u {
		cam_ts_RectangularShape_t	 rectangular;
		cam_ts_CircularShape_t	 circular;
		cam_ts_PolygonalShape_t	 polygonal;
		cam_ts_EllipticalShape_t	 elliptical;
		cam_ts_RadialShape_t	 radial;
		cam_ts_RadialShapes_t	 radialShapes;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_Shape_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_Shape;
extern asn_CHOICE_specifics_t asn_SPC_cam_ts_Shape_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_Shape_1[6];
extern asn_per_constraints_t asn_PER_type_cam_ts_Shape_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_Shape_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
