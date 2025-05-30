/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Descriptions"
 * 	found in "/input/DENM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_DENM_H_
#define	_denm_DENM_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_coding/denm_ItsPduHeader.h"
#include "etsi_its_denm_coding/denm_DecentralizedEnvironmentalNotificationMessage.h"
#include <etsi_its_denm_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* denm_DENM */
typedef struct denm_DENM {
	denm_ItsPduHeader_t	 header;
	denm_DecentralizedEnvironmentalNotificationMessage_t	 denm;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_DENM_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_DENM;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_DENM_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
