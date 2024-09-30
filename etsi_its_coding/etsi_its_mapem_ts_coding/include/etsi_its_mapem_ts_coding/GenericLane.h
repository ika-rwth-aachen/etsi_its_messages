/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/LaneID.h"
#include "etsi_its_mapem_ts_coding/DescriptiveName.h"
#include "etsi_its_mapem_ts_coding/ApproachID.h"
#include "etsi_its_mapem_ts_coding/LaneAttributes.h"
#include "etsi_its_mapem_ts_coding/AllowedManeuvers.h"
#include "etsi_its_mapem_ts_coding/NodeListXY.h"
#include <etsi_its_mapem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#ifndef	_GenericLane_H_
#define	_GenericLane_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ConnectsToList;
struct OverlayLaneList;
struct Reg_GenericLane;

/* GenericLane */
typedef struct GenericLane {
	LaneID_t	 laneID;
	DescriptiveName_t	*name;	/* OPTIONAL */
	ApproachID_t	*ingressApproach;	/* OPTIONAL */
	ApproachID_t	*egressApproach;	/* OPTIONAL */
	LaneAttributes_t	 laneAttributes;
	AllowedManeuvers_t	*maneuvers;	/* OPTIONAL */
	NodeListXY_t	 nodeList;
	struct ConnectsToList	*connectsTo;	/* OPTIONAL */
	struct OverlayLaneList	*overlays;	/* OPTIONAL */
	struct GenericLane__regional {
		A_SEQUENCE_OF(struct Reg_GenericLane) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} GenericLane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_GenericLane;
extern asn_SEQUENCE_specifics_t asn_SPC_GenericLane_specs_1;
extern asn_TYPE_member_t asn_MBR_GenericLane_1[10];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_ts_coding/ConnectsToList.h"
#include "etsi_its_mapem_ts_coding/OverlayLaneList.h"
#include "etsi_its_mapem_ts_coding/RegionalExtension.h"

#endif	/* _GenericLane_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
