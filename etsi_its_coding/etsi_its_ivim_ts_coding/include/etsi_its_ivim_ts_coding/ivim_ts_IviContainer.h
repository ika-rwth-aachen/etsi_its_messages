/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_IviContainer_H_
#define	_ivim_ts_IviContainer_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_GeographicLocationContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_GeneralIviContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadConfigurationContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_TextContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LayoutContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_AutomatedVehicleContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_MapLocationContainer.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadSurfaceContainer.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>
#include <etsi_its_ivim_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_IviContainer_PR {
	ivim_ts_IviContainer_PR_NOTHING,	/* No components present */
	ivim_ts_IviContainer_PR_glc,
	ivim_ts_IviContainer_PR_giv,
	ivim_ts_IviContainer_PR_rcc,
	ivim_ts_IviContainer_PR_tc,
	ivim_ts_IviContainer_PR_lac,
	/* Extensions may appear below */
	ivim_ts_IviContainer_PR_ext1
} ivim_ts_IviContainer_PR;

/* ivim_ts_IviContainer */
typedef struct ivim_ts_IviContainer {
	ivim_ts_IviContainer_PR present;
	union ivim_ts_IviContainer_u {
		ivim_ts_GeographicLocationContainer_t	 glc;
		ivim_ts_GeneralIviContainer_t	 giv;
		ivim_ts_RoadConfigurationContainer_t	 rcc;
		ivim_ts_TextContainer_t	 tc;
		ivim_ts_LayoutContainer_t	 lac;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		struct ivim_ts_IviContainer__ext1 {
			ivim_ts_AutomatedVehicleContainer_t	 avc;
			ivim_ts_MapLocationContainer_t	 mlc;
			ivim_ts_RoadSurfaceContainer_t	 rsc;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *ext1;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_IviContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_IviContainer;
extern asn_CHOICE_specifics_t asn_SPC_ivim_ts_IviContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_IviContainer_1[6];
extern asn_per_constraints_t asn_PER_type_ivim_ts_IviContainer_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_IviContainer_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
