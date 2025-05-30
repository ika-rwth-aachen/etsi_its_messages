/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mapem_ts_GenericLane_H_
#define	_mapem_ts_GenericLane_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/mapem_ts_LaneID.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_DescriptiveName.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_ApproachID.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_LaneAttributes.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_AllowedManeuvers.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_NodeListXY.h"
#include <etsi_its_mapem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mapem_ts_ConnectsToList;
struct mapem_ts_OverlayLaneList;
struct mapem_ts_Reg_GenericLane;

/* mapem_ts_GenericLane */
typedef struct mapem_ts_GenericLane {
	mapem_ts_LaneID_t	 laneID;
	mapem_ts_DescriptiveName_t	*name;	/* OPTIONAL */
	mapem_ts_ApproachID_t	*ingressApproach;	/* OPTIONAL */
	mapem_ts_ApproachID_t	*egressApproach;	/* OPTIONAL */
	mapem_ts_LaneAttributes_t	 laneAttributes;
	mapem_ts_AllowedManeuvers_t	*maneuvers;	/* OPTIONAL */
	mapem_ts_NodeListXY_t	 nodeList;
	struct mapem_ts_ConnectsToList	*connectsTo;	/* OPTIONAL */
	struct mapem_ts_OverlayLaneList	*overlays;	/* OPTIONAL */
	struct mapem_ts_GenericLane__regional {
		A_SEQUENCE_OF(struct mapem_ts_Reg_GenericLane) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mapem_ts_GenericLane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_GenericLane;
extern asn_SEQUENCE_specifics_t asn_SPC_mapem_ts_GenericLane_specs_1;
extern asn_TYPE_member_t asn_MBR_mapem_ts_GenericLane_1[10];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_ts_coding/mapem_ts_ConnectsToList.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_OverlayLaneList.h"
#include "etsi_its_mapem_ts_coding/mapem_ts_RegionalExtension.h"

#endif	/* _mapem_ts_GenericLane_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
