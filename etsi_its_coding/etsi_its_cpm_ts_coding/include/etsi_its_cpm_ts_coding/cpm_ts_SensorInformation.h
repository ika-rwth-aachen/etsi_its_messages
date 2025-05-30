/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-SensorInformationContainer"
 * 	found in "/input/CPM-SensorInformationContainer.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cpm_ts_SensorInformation_H_
#define	_cpm_ts_SensorInformation_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cpm_ts_coding/cpm_ts_Identifier1B.h"
#include "etsi_its_cpm_ts_coding/cpm_ts_SensorType.h"
#include "etsi_its_cpm_ts_coding/cpm_ts_ConfidenceLevel.h"
#include <etsi_its_cpm_ts_coding/BOOLEAN.h>
#include <etsi_its_cpm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cpm_ts_Shape;

/* cpm_ts_SensorInformation */
typedef struct cpm_ts_SensorInformation {
	cpm_ts_Identifier1B_t	 sensorId;
	cpm_ts_SensorType_t	 sensorType;
	struct cpm_ts_Shape	*perceptionRegionShape;	/* OPTIONAL */
	cpm_ts_ConfidenceLevel_t	*perceptionRegionConfidence;	/* OPTIONAL */
	BOOLEAN_t	 shadowingApplies;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_ts_SensorInformation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_SensorInformation;
extern asn_SEQUENCE_specifics_t asn_SPC_cpm_ts_SensorInformation_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_ts_SensorInformation_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cpm_ts_coding/cpm_ts_Shape.h"

#endif	/* _cpm_ts_SensorInformation_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>
