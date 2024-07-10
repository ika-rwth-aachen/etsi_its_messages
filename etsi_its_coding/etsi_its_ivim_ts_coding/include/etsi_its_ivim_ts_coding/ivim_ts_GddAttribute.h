/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "GDD"
 * 	found in "/input/ISO14823-missing.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_GddAttribute_H_
#define	_ivim_ts_GddAttribute_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-applicablePeriod.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-exemptedApplicablePeriod.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-directionalFlowOfLane.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-applicableVehicleDimensions.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-speedLimits.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-rateOfIncline.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-distanceBetweenVehicles.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-section.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-numberOfLane.h"
#include <etsi_its_ivim_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_GddAttribute_PR {
	ivim_ts_GddAttribute_PR_NOTHING,	/* No components present */
	ivim_ts_GddAttribute_PR_dtm,
	ivim_ts_GddAttribute_PR_edt,
	ivim_ts_GddAttribute_PR_dfl,
	ivim_ts_GddAttribute_PR_ved,
	ivim_ts_GddAttribute_PR_spe,
	ivim_ts_GddAttribute_PR_roi,
	ivim_ts_GddAttribute_PR_dbv,
	ivim_ts_GddAttribute_PR_ddd,
	ivim_ts_GddAttribute_PR_set,
	ivim_ts_GddAttribute_PR_nol
} ivim_ts_GddAttribute_PR;

/* Forward declarations */
struct ivim_ts_InternationalSign_destinationInformation;

/* ivim_ts_GddAttribute */
typedef struct ivim_ts_GddAttribute {
	ivim_ts_GddAttribute_PR present;
	union ivim_ts_GddAttribute_u {
		ivim_ts_InternationalSign_applicablePeriod_t	 dtm;
		ivim_ts_InternationalSign_exemptedApplicablePeriod_t	 edt;
		ivim_ts_InternationalSign_directionalFlowOfLane_t	 dfl;
		ivim_ts_InternationalSign_applicableVehicleDimensions_t	 ved;
		ivim_ts_InternationalSign_speedLimits_t	 spe;
		ivim_ts_InternationalSign_rateOfIncline_t	 roi;
		ivim_ts_InternationalSign_distanceBetweenVehicles_t	 dbv;
		struct ivim_ts_InternationalSign_destinationInformation	*ddd;
		ivim_ts_InternationalSign_section_t	 set;
		ivim_ts_InternationalSign_numberOfLane_t	 nol;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_GddAttribute_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_GddAttribute;
extern asn_CHOICE_specifics_t asn_SPC_ivim_ts_GddAttribute_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_GddAttribute_1[10];
extern asn_per_constraints_t asn_PER_type_ivim_ts_GddAttribute_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-destinationInformation.h"

#endif	/* _ivim_ts_GddAttribute_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
