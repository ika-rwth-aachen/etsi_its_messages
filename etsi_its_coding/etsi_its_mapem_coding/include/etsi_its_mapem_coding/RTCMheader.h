/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_coding/GNSSstatus.h"
#include "etsi_its_mapem_coding/AntennaOffsetSet.h"
#include <etsi_its_mapem_coding/constr_SEQUENCE.h>
#ifndef	_RTCMheader_H_
#define	_RTCMheader_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RTCMheader */
typedef struct RTCMheader {
	GNSSstatus_t	 status;
	AntennaOffsetSet_t	 offsetSet;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RTCMheader_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RTCMheader;
extern asn_SEQUENCE_specifics_t asn_SPC_RTCMheader_specs_1;
extern asn_TYPE_member_t asn_MBR_RTCMheader_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RTCMheader_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
