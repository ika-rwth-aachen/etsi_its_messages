/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-PER`
 */

#ifndef	_MapData_H_
#define	_MapData_H_


#include <etsi_its_mapem_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_mapem_coding/MinuteOfTheYear.h"
#include "etsi_its_mapem_coding/MsgCount.h"
#include "etsi_its_mapem_coding/LayerType.h"
#include "etsi_its_mapem_coding/LayerID.h"
#include <etsi_its_mapem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct IntersectionGeometryList;
struct RoadSegmentList;
struct DataParameters;
struct RestrictionClassList;
struct RegionalExtension;

/* MapData */
typedef struct MapData {
	MinuteOfTheYear_t	*timeStamp	/* OPTIONAL */;
	MsgCount_t	 msgIssueRevision;
	LayerType_t	*layerType	/* OPTIONAL */;
	LayerID_t	*layerID	/* OPTIONAL */;
	struct IntersectionGeometryList	*intersections	/* OPTIONAL */;
	struct RoadSegmentList	*roadSegments	/* OPTIONAL */;
	struct DataParameters	*dataParameters	/* OPTIONAL */;
	struct RestrictionClassList	*restrictionList	/* OPTIONAL */;
	struct MapData__regional {
		A_SEQUENCE_OF(struct RegionalExtension) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MapData_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MapData;
extern asn_SEQUENCE_specifics_t asn_SPC_MapData_specs_1;
extern asn_TYPE_member_t asn_MBR_MapData_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_coding/IntersectionGeometryList.h"
#include "etsi_its_mapem_coding/RoadSegmentList.h"
#include "etsi_its_mapem_coding/DataParameters.h"
#include "etsi_its_mapem_coding/RestrictionClassList.h"
#include "etsi_its_mapem_coding/RegionalExtension.h"

#endif	/* _MapData_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>