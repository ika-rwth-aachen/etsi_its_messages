/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_MapemElementReference_H_
#define	_mcm_uulm_MapemElementReference_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mcm_uulm_MapReference;
struct mcm_uulm_MapemLaneList;
struct mcm_uulm_MapemConnectionList;

/* mcm_uulm_MapemElementReference */
typedef struct mcm_uulm_MapemElementReference {
	struct mcm_uulm_MapReference	*mapReference;	/* OPTIONAL */
	struct mcm_uulm_MapemLaneList	*laneIds;	/* OPTIONAL */
	struct mcm_uulm_MapemConnectionList	*connectionIds;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_MapemElementReference_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_MapemElementReference;
extern asn_SEQUENCE_specifics_t asn_SPC_mcm_uulm_MapemElementReference_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_uulm_MapemElementReference_1[3];
extern asn_per_constraints_t asn_PER_type_mcm_uulm_MapemElementReference_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_MapReference.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_MapemLaneList.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_MapemConnectionList.h"

#endif	/* _mcm_uulm_MapemElementReference_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
