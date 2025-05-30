/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_SteeringWheelAngle_H_
#define	_denm_SteeringWheelAngle_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_coding/denm_SteeringWheelAngleValue.h"
#include "etsi_its_denm_coding/denm_SteeringWheelAngleConfidence.h"
#include <etsi_its_denm_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* denm_SteeringWheelAngle */
typedef struct denm_SteeringWheelAngle {
	denm_SteeringWheelAngleValue_t	 steeringWheelAngleValue;
	denm_SteeringWheelAngleConfidence_t	 steeringWheelAngleConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_SteeringWheelAngle_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_SteeringWheelAngle;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_SteeringWheelAngle_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
