/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_denm_coding/NativeInteger.h>
#include "etsi_its_denm_coding/StationID.h"
#include <etsi_its_denm_coding/constr_SEQUENCE.h>
#ifndef	_ItsPduHeader_H_
#define	_ItsPduHeader_H_


#include <etsi_its_denm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ItsPduHeader__messageID {
	ItsPduHeader__messageID_denm	= 1,
	ItsPduHeader__messageID_cam	= 2,
	ItsPduHeader__messageID_poi	= 3,
	ItsPduHeader__messageID_spatem	= 4,
	ItsPduHeader__messageID_mapem	= 5,
	ItsPduHeader__messageID_ivim	= 6,
	ItsPduHeader__messageID_ev_rsr	= 7,
	ItsPduHeader__messageID_tistpgtransaction	= 8,
	ItsPduHeader__messageID_srem	= 9,
	ItsPduHeader__messageID_ssem	= 10,
	ItsPduHeader__messageID_evcsn	= 11,
	ItsPduHeader__messageID_saem	= 12,
	ItsPduHeader__messageID_rtcmem	= 13
} e_ItsPduHeader__messageID;

/* ItsPduHeader */
typedef struct ItsPduHeader {
	long	 protocolVersion;
	long	 messageID;
	StationID_t	 stationID;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ItsPduHeader_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ItsPduHeader;
extern asn_SEQUENCE_specifics_t asn_SPC_ItsPduHeader_specs_1;
extern asn_TYPE_member_t asn_MBR_ItsPduHeader_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _ItsPduHeader_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
