/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_coding/MsgCount.h"
#include "etsi_its_mapem_coding/RTCM-Revision.h"
#include "etsi_its_mapem_coding/MinuteOfTheYear.h"
#include "etsi_its_mapem_coding/RTCMmessageList.h"
#include <etsi_its_mapem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE.h>
#ifndef	_RTCMcorrections_H_
#define	_RTCMcorrections_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct FullPositionVector;
struct RTCMheader;
struct Reg_RTCMcorrections;

/* RTCMcorrections */
typedef struct RTCMcorrections {
	MsgCount_t	 msgCnt;
	RTCM_Revision_t	 rev;
	MinuteOfTheYear_t	*timeStamp;	/* OPTIONAL */
	struct FullPositionVector	*anchorPoint;	/* OPTIONAL */
	struct RTCMheader	*rtcmHeader;	/* OPTIONAL */
	RTCMmessageList_t	 msgs;
	struct RTCMcorrections__regional {
		A_SEQUENCE_OF(struct Reg_RTCMcorrections) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RTCMcorrections_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RTCMcorrections;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_coding/FullPositionVector.h"
#include "etsi_its_mapem_coding/RTCMheader.h"
#include "etsi_its_mapem_coding/RegionalExtension.h"

#endif	/* _RTCMcorrections_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
