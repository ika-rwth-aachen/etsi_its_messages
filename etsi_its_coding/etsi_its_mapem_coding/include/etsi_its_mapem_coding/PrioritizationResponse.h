/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_coding/StationID.h"
#include "etsi_its_mapem_coding/PrioritizationResponseStatus.h"
#include "etsi_its_mapem_coding/SignalGroupID.h"
#include <etsi_its_mapem_coding/constr_SEQUENCE.h>
#ifndef	_PrioritizationResponse_H_
#define	_PrioritizationResponse_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PrioritizationResponse */
typedef struct PrioritizationResponse {
	StationID_t	 stationID;
	PrioritizationResponseStatus_t	 priorState;
	SignalGroupID_t	 signalGroup;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PrioritizationResponse_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PrioritizationResponse;
extern asn_SEQUENCE_specifics_t asn_SPC_PrioritizationResponse_specs_1;
extern asn_TYPE_member_t asn_MBR_PrioritizationResponse_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _PrioritizationResponse_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
