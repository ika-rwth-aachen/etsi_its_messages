/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_SignalRequest_H_
#define	_ivim_ts_SignalRequest_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_IntersectionReferenceID.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RequestID.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_PriorityRequestType.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_IntersectionAccessPoint.h"
#include <etsi_its_ivim_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ivim_ts_IntersectionAccessPoint;
struct ivim_ts_Reg_SignalRequest;

/* ivim_ts_SignalRequest */
typedef struct ivim_ts_SignalRequest {
	ivim_ts_IntersectionReferenceID_t	 id;
	ivim_ts_RequestID_t	 requestID;
	ivim_ts_PriorityRequestType_t	 requestType;
	ivim_ts_IntersectionAccessPoint_t	 inBoundLane;
	struct ivim_ts_IntersectionAccessPoint	*outBoundLane;	/* OPTIONAL */
	struct ivim_ts_SignalRequest__regional {
		A_SEQUENCE_OF(struct ivim_ts_Reg_SignalRequest) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_SignalRequest_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_SignalRequest;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_SignalRequest_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_SignalRequest_1[6];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_ivim_ts_coding/ivim_ts_IntersectionAccessPoint.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RegionalExtension.h"

#endif	/* _ivim_ts_SignalRequest_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
