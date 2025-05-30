/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_MapemElementReference_H_
#define	_cam_ts_MapemElementReference_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cam_ts_MapReference;
struct cam_ts_MapemLaneList;
struct cam_ts_MapemConnectionList;

/* cam_ts_MapemElementReference */
typedef struct cam_ts_MapemElementReference {
	struct cam_ts_MapReference	*mapReference;	/* OPTIONAL */
	struct cam_ts_MapemLaneList	*laneIds;	/* OPTIONAL */
	struct cam_ts_MapemConnectionList	*connectionIds;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_MapemElementReference_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_MapemElementReference;
extern asn_SEQUENCE_specifics_t asn_SPC_cam_ts_MapemElementReference_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_MapemElementReference_1[3];
extern asn_per_constraints_t asn_PER_type_cam_ts_MapemElementReference_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cam_ts_coding/cam_ts_MapReference.h"
#include "etsi_its_cam_ts_coding/cam_ts_MapemLaneList.h"
#include "etsi_its_cam_ts_coding/cam_ts_MapemConnectionList.h"

#endif	/* _cam_ts_MapemElementReference_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
