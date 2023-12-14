/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/BIT_STRING.h>
#ifndef	_IntersectionStatusObject_H_
#define	_IntersectionStatusObject_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum IntersectionStatusObject {
	IntersectionStatusObject_manualControlIsEnabled	= 0,
	IntersectionStatusObject_stopTimeIsActivated	= 1,
	IntersectionStatusObject_failureFlash	= 2,
	IntersectionStatusObject_preemptIsActive	= 3,
	IntersectionStatusObject_signalPriorityIsActive	= 4,
	IntersectionStatusObject_fixedTimeOperation	= 5,
	IntersectionStatusObject_trafficDependentOperation	= 6,
	IntersectionStatusObject_standbyOperation	= 7,
	IntersectionStatusObject_failureMode	= 8,
	IntersectionStatusObject_off	= 9,
	IntersectionStatusObject_recentMAPmessageUpdate	= 10,
	IntersectionStatusObject_recentChangeInMAPassignedLanesIDsUsed	= 11,
	IntersectionStatusObject_noValidMAPisAvailableAtThisTime	= 12,
	IntersectionStatusObject_noValidSPATisAvailableAtThisTime	= 13
} e_IntersectionStatusObject;

/* IntersectionStatusObject */
typedef BIT_STRING_t	 IntersectionStatusObject_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_IntersectionStatusObject_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_IntersectionStatusObject;
asn_struct_free_f IntersectionStatusObject_free;
asn_struct_print_f IntersectionStatusObject_print;
asn_constr_check_f IntersectionStatusObject_constraint;
ber_type_decoder_f IntersectionStatusObject_decode_ber;
der_type_encoder_f IntersectionStatusObject_encode_der;
xer_type_decoder_f IntersectionStatusObject_decode_xer;
xer_type_encoder_f IntersectionStatusObject_encode_xer;
jer_type_encoder_f IntersectionStatusObject_encode_jer;
oer_type_decoder_f IntersectionStatusObject_decode_oer;
oer_type_encoder_f IntersectionStatusObject_encode_oer;
per_type_decoder_f IntersectionStatusObject_decode_uper;
per_type_encoder_f IntersectionStatusObject_encode_uper;
per_type_decoder_f IntersectionStatusObject_decode_aper;
per_type_encoder_f IntersectionStatusObject_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _IntersectionStatusObject_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
