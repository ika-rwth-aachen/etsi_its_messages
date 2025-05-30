/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_PathPointPredicted_H_
#define	_denm_ts_PathPointPredicted_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_ts_coding/denm_ts_DeltaLatitude.h"
#include "etsi_its_denm_ts_coding/denm_ts_DeltaLongitude.h"
#include "etsi_its_denm_ts_coding/denm_ts_DeltaAltitude.h"
#include "etsi_its_denm_ts_coding/denm_ts_AltitudeConfidence.h"
#include "etsi_its_denm_ts_coding/denm_ts_StandardLength9b.h"
#include <etsi_its_denm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct denm_ts_PosConfidenceEllipse;
struct denm_ts_PathDeltaTimeChoice;

/* denm_ts_PathPointPredicted */
typedef struct denm_ts_PathPointPredicted {
	denm_ts_DeltaLatitude_t	 deltaLatitude;
	denm_ts_DeltaLongitude_t	 deltaLongitude;
	struct denm_ts_PosConfidenceEllipse	*horizontalPositionConfidence;	/* OPTIONAL */
	denm_ts_DeltaAltitude_t	*deltaAltitude;	/* DEFAULT 12800 */
	denm_ts_AltitudeConfidence_t	*altitudeConfidence;	/* DEFAULT 15 */
	struct denm_ts_PathDeltaTimeChoice	*pathDeltaTime;	/* OPTIONAL */
	denm_ts_StandardLength9b_t	*symmetricAreaOffset;	/* OPTIONAL */
	denm_ts_StandardLength9b_t	*asymmetricAreaOffset;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_ts_PathPointPredicted_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_PathPointPredicted;
extern asn_SEQUENCE_specifics_t asn_SPC_denm_ts_PathPointPredicted_specs_1;
extern asn_TYPE_member_t asn_MBR_denm_ts_PathPointPredicted_1[8];
extern asn_per_constraints_t asn_PER_type_denm_ts_PathPointPredicted_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_denm_ts_coding/denm_ts_PosConfidenceEllipse.h"
#include "etsi_its_denm_ts_coding/denm_ts_PathDeltaTimeChoice.h"

#endif	/* _denm_ts_PathPointPredicted_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
