/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_DigitalMap_H_
#define	_spatem_ts_DigitalMap_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct spatem_ts_ReferencePosition;

/* spatem_ts_DigitalMap */
typedef struct spatem_ts_DigitalMap {
	A_SEQUENCE_OF(struct spatem_ts_ReferencePosition) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} spatem_ts_DigitalMap_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_DigitalMap;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_ts_coding/spatem_ts_ReferencePosition.h"

#endif	/* _spatem_ts_DigitalMap_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
