/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_LateralAcceleration_H_
#define	_denm_LateralAcceleration_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_coding/denm_LateralAccelerationValue.h"
#include "etsi_its_denm_coding/denm_AccelerationConfidence.h"
#include <etsi_its_denm_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* denm_LateralAcceleration */
typedef struct denm_LateralAcceleration {
	denm_LateralAccelerationValue_t	 lateralAccelerationValue;
	denm_AccelerationConfidence_t	 lateralAccelerationConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_LateralAcceleration_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_LateralAcceleration;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_LateralAcceleration_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
