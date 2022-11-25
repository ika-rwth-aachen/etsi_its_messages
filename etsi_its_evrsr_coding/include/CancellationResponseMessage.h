/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EV-RechargingSpotReservation-PDU-Descriptions"
 * 	found in "/input/EV-RSR-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-PER`
 */

#ifndef	_CancellationResponseMessage_H_
#define	_CancellationResponseMessage_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Reservation-ID.h"
#include "CancellationResponseCode.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CancellationResponseMessage */
typedef struct CancellationResponseMessage {
	Reservation_ID_t	 reservation_ID;
	CancellationResponseCode_t	 cancellationResponseCode;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CancellationResponseMessage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CancellationResponseMessage;
extern asn_SEQUENCE_specifics_t asn_SPC_CancellationResponseMessage_specs_1;
extern asn_TYPE_member_t asn_MBR_CancellationResponseMessage_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _CancellationResponseMessage_H_ */
#include <asn_internal.h>
