/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_spatem_coding/DescriptiveName.h"
#include "etsi_its_spatem_coding/SignalGroupID.h"
#include "etsi_its_spatem_coding/MovementEventList.h"
#include <etsi_its_spatem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_spatem_coding/constr_SEQUENCE.h>
#ifndef	_MovementState_H_
#define	_MovementState_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ManeuverAssistList;
struct Reg_MovementState;

/* MovementState */
typedef struct MovementState {
	DescriptiveName_t	*movementName;	/* OPTIONAL */
	SignalGroupID_t	 signalGroup;
	MovementEventList_t	 state_time_speed;
	struct ManeuverAssistList	*maneuverAssistList;	/* OPTIONAL */
	struct MovementState__regional {
		A_SEQUENCE_OF(struct Reg_MovementState) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MovementState_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MovementState;
extern asn_SEQUENCE_specifics_t asn_SPC_MovementState_specs_1;
extern asn_TYPE_member_t asn_MBR_MovementState_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_coding/ManeuverAssistList.h"
#include "etsi_its_spatem_coding/RegionalExtension.h"

#endif	/* _MovementState_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>