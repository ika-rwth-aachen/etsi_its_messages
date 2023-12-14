/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_coding/constr_SEQUENCE_OF.h>
#ifndef	_ItineraryPath_H_
#define	_ItineraryPath_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ReferencePosition;

/* ItineraryPath */
typedef struct ItineraryPath {
	A_SEQUENCE_OF(struct ReferencePosition) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ItineraryPath_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ItineraryPath;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_coding/ReferencePosition.h"

#endif	/* _ItineraryPath_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
