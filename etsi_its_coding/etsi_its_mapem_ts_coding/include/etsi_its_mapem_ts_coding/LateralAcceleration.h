/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/LateralAccelerationValue.h"
#include "etsi_its_mapem_ts_coding/AccelerationConfidence.h"
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#ifndef	_LateralAcceleration_H_
#define	_LateralAcceleration_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LateralAcceleration */
typedef struct LateralAcceleration {
	LateralAccelerationValue_t	 lateralAccelerationValue;
	AccelerationConfidence_t	 lateralAccelerationConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LateralAcceleration_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LateralAcceleration;

#ifdef __cplusplus
}
#endif

#endif	/* _LateralAcceleration_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
