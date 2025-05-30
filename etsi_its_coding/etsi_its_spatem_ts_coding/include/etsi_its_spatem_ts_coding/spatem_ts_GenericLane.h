/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_GenericLane_H_
#define	_spatem_ts_GenericLane_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_spatem_ts_coding/spatem_ts_LaneID.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_DescriptiveName.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_ApproachID.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_LaneAttributes.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_AllowedManeuvers.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_NodeListXY.h"
#include <etsi_its_spatem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct spatem_ts_ConnectsToList;
struct spatem_ts_OverlayLaneList;
struct spatem_ts_Reg_GenericLane;

/* spatem_ts_GenericLane */
typedef struct spatem_ts_GenericLane {
	spatem_ts_LaneID_t	 laneID;
	spatem_ts_DescriptiveName_t	*name;	/* OPTIONAL */
	spatem_ts_ApproachID_t	*ingressApproach;	/* OPTIONAL */
	spatem_ts_ApproachID_t	*egressApproach;	/* OPTIONAL */
	spatem_ts_LaneAttributes_t	 laneAttributes;
	spatem_ts_AllowedManeuvers_t	*maneuvers;	/* OPTIONAL */
	spatem_ts_NodeListXY_t	 nodeList;
	struct spatem_ts_ConnectsToList	*connectsTo;	/* OPTIONAL */
	struct spatem_ts_OverlayLaneList	*overlays;	/* OPTIONAL */
	struct spatem_ts_GenericLane__regional {
		A_SEQUENCE_OF(struct spatem_ts_Reg_GenericLane) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} spatem_ts_GenericLane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_GenericLane;
extern asn_SEQUENCE_specifics_t asn_SPC_spatem_ts_GenericLane_specs_1;
extern asn_TYPE_member_t asn_MBR_spatem_ts_GenericLane_1[10];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_ts_coding/spatem_ts_ConnectsToList.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_OverlayLaneList.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_RegionalExtension.h"

#endif	/* _spatem_ts_GenericLane_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
