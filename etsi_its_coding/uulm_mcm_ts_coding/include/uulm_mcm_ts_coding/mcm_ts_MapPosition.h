/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_MapPosition_H_
#define	_mcm_ts_MapPosition_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "uulm_mcm_ts_coding/mcm_ts_Identifier1B.h"
#include <uulm_mcm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mcm_ts_MapReference;
struct mcm_ts_LongitudinalLanePosition;

/* mcm_ts_MapPosition */
typedef struct mcm_ts_MapPosition {
	struct mcm_ts_MapReference	*mapReference;	/* OPTIONAL */
	mcm_ts_Identifier1B_t	*laneId;	/* OPTIONAL */
	mcm_ts_Identifier1B_t	*connectionId;	/* OPTIONAL */
	struct mcm_ts_LongitudinalLanePosition	*longitudinalLanePosition;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_ts_MapPosition_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_MapPosition;
extern asn_SEQUENCE_specifics_t asn_SPC_mcm_ts_MapPosition_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_ts_MapPosition_1[4];
extern asn_per_constraints_t asn_PER_type_mcm_ts_MapPosition_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "uulm_mcm_ts_coding/mcm_ts_MapReference.h"
#include "uulm_mcm_ts_coding/mcm_ts_LongitudinalLanePosition.h"

#endif	/* _mcm_ts_MapPosition_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
