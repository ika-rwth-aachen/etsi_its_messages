/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Description"
 * 	found in "/input/DENM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_AlacarteContainer_H_
#define	_denm_ts_AlacarteContainer_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_ts_coding/denm_ts_LanePosition.h"
#include "etsi_its_denm_ts_coding/denm_ts_Temperature.h"
#include "etsi_its_denm_ts_coding/denm_ts_PositioningSolutionType.h"
#include <etsi_its_denm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct denm_ts_ImpactReductionContainer;
struct denm_ts_RoadWorksContainerExtended;
struct denm_ts_StationaryVehicleContainer;
struct denm_ts_RoadConfigurationContainer;
struct denm_ts_PreCrashContainer;

/* denm_ts_AlacarteContainer */
typedef struct denm_ts_AlacarteContainer {
	denm_ts_LanePosition_t	*lanePosition;	/* OPTIONAL */
	struct denm_ts_ImpactReductionContainer	*impactReduction;	/* OPTIONAL */
	denm_ts_Temperature_t	*externalTemperature;	/* OPTIONAL */
	struct denm_ts_RoadWorksContainerExtended	*roadWorks;	/* OPTIONAL */
	denm_ts_PositioningSolutionType_t	*positioningSolution;	/* OPTIONAL */
	struct denm_ts_StationaryVehicleContainer	*stationaryVehicle;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct denm_ts_AlacarteContainer__ext1 {
		struct denm_ts_RoadConfigurationContainer	*roadConfiguration;	/* OPTIONAL */
		struct denm_ts_PreCrashContainer	*preCrash;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_ts_AlacarteContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_AlacarteContainer;
extern asn_SEQUENCE_specifics_t asn_SPC_denm_ts_AlacarteContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_denm_ts_AlacarteContainer_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_denm_ts_coding/denm_ts_ImpactReductionContainer.h"
#include "etsi_its_denm_ts_coding/denm_ts_RoadWorksContainerExtended.h"
#include "etsi_its_denm_ts_coding/denm_ts_StationaryVehicleContainer.h"
#include "etsi_its_denm_ts_coding/denm_ts_RoadConfigurationContainer.h"
#include "etsi_its_denm_ts_coding/denm_ts_PreCrashContainer.h"

#endif	/* _denm_ts_AlacarteContainer_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
