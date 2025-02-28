/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "MCM-PDU-Descriptions"
 * 	found in "/input/TS103561_LUKAS_MCM.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_ManeuverCoordinationMessage_H_
#define	_mcm_ts_ManeuverCoordinationMessage_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "uulm_mcm_ts_coding/mcm_ts_GenerationDeltaTime.h"
#include "uulm_mcm_ts_coding/mcm_ts_McmParameters.h"
#include <uulm_mcm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mcm_ts_ManeuverCoordinationMessage */
typedef struct mcm_ts_ManeuverCoordinationMessage {
	mcm_ts_GenerationDeltaTime_t	 generationDeltaTime;
	mcm_ts_McmParameters_t	 mcmParameters;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_ts_ManeuverCoordinationMessage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_ManeuverCoordinationMessage;
extern asn_SEQUENCE_specifics_t asn_SPC_mcm_ts_ManeuverCoordinationMessage_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_ts_ManeuverCoordinationMessage_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_ManeuverCoordinationMessage_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
