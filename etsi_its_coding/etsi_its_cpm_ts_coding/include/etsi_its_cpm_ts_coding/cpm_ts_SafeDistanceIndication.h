/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cpm_ts_SafeDistanceIndication_H_
#define	_cpm_ts_SafeDistanceIndication_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cpm_ts_coding/cpm_ts_StationId.h"
#include "etsi_its_cpm_ts_coding/cpm_ts_SafeDistanceIndicator.h"
#include "etsi_its_cpm_ts_coding/cpm_ts_DeltaTimeTenthOfSecond.h"
#include <etsi_its_cpm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* cpm_ts_SafeDistanceIndication */
typedef struct cpm_ts_SafeDistanceIndication {
	cpm_ts_StationId_t	*subjectStation;	/* OPTIONAL */
	cpm_ts_SafeDistanceIndicator_t	 safeDistanceIndicator;
	cpm_ts_DeltaTimeTenthOfSecond_t	*timeToCollision;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_ts_SafeDistanceIndication_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_SafeDistanceIndication;
extern asn_SEQUENCE_specifics_t asn_SPC_cpm_ts_SafeDistanceIndication_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_ts_SafeDistanceIndication_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_ts_SafeDistanceIndication_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>
