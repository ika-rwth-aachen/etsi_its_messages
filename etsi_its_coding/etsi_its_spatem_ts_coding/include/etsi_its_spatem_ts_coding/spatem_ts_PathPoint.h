/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_PathPoint_H_
#define	_spatem_ts_PathPoint_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_spatem_ts_coding/spatem_ts_DeltaReferencePosition.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_PathDeltaTime.h"
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* spatem_ts_PathPoint */
typedef struct spatem_ts_PathPoint {
	spatem_ts_DeltaReferencePosition_t	 pathPosition;
	spatem_ts_PathDeltaTime_t	*pathDeltaTime;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} spatem_ts_PathPoint_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_PathPoint;
extern asn_SEQUENCE_specifics_t asn_SPC_spatem_ts_PathPoint_specs_1;
extern asn_TYPE_member_t asn_MBR_spatem_ts_PathPoint_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_PathPoint_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
