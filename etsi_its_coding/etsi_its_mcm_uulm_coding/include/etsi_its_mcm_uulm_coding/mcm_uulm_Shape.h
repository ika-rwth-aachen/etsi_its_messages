/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_Shape_H_
#define	_mcm_uulm_Shape_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_RectangularShape.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_CircularShape.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_PolygonalShape.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_EllipticalShape.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_RadialShape.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_RadialShapes.h"
#include <etsi_its_mcm_uulm_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_Shape_PR {
	mcm_uulm_Shape_PR_NOTHING,	/* No components present */
	mcm_uulm_Shape_PR_rectangular,
	mcm_uulm_Shape_PR_circular,
	mcm_uulm_Shape_PR_polygonal,
	mcm_uulm_Shape_PR_elliptical,
	mcm_uulm_Shape_PR_radial,
	mcm_uulm_Shape_PR_radialShapes
	/* Extensions may appear below */
	
} mcm_uulm_Shape_PR;

/* mcm_uulm_Shape */
typedef struct mcm_uulm_Shape {
	mcm_uulm_Shape_PR present;
	union mcm_uulm_Shape_u {
		mcm_uulm_RectangularShape_t	 rectangular;
		mcm_uulm_CircularShape_t	 circular;
		mcm_uulm_PolygonalShape_t	 polygonal;
		mcm_uulm_EllipticalShape_t	 elliptical;
		mcm_uulm_RadialShape_t	 radial;
		mcm_uulm_RadialShapes_t	 radialShapes;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_Shape_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_Shape;
extern asn_CHOICE_specifics_t asn_SPC_mcm_uulm_Shape_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_uulm_Shape_1[6];
extern asn_per_constraints_t asn_PER_type_mcm_uulm_Shape_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_Shape_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
