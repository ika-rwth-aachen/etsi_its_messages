/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_LongitudinalAcceleration_H_
#define	_mcm_ts_LongitudinalAcceleration_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "uulm_mcm_ts_coding/mcm_ts_LongitudinalAccelerationValue.h"
#include "uulm_mcm_ts_coding/mcm_ts_AccelerationConfidence.h"
#include <uulm_mcm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mcm_ts_LongitudinalAcceleration */
typedef struct mcm_ts_LongitudinalAcceleration {
	mcm_ts_LongitudinalAccelerationValue_t	 longitudinalAccelerationValue;
	mcm_ts_AccelerationConfidence_t	 longitudinalAccelerationConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_ts_LongitudinalAcceleration_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_LongitudinalAcceleration;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_LongitudinalAcceleration_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
