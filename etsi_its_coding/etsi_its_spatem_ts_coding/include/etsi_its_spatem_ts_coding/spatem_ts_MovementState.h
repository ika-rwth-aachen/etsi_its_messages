/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_MovementState_H_
#define	_spatem_ts_MovementState_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_spatem_ts_coding/spatem_ts_DescriptiveName.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_SignalGroupID.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_MovementEventList.h"
#include <etsi_its_spatem_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct spatem_ts_ManeuverAssistList;
struct spatem_ts_Reg_MovementState;

/* spatem_ts_MovementState */
typedef struct spatem_ts_MovementState {
	spatem_ts_DescriptiveName_t	*movementName;	/* OPTIONAL */
	spatem_ts_SignalGroupID_t	 signalGroup;
	spatem_ts_MovementEventList_t	 state_time_speed;
	struct spatem_ts_ManeuverAssistList	*maneuverAssistList;	/* OPTIONAL */
	struct spatem_ts_MovementState__regional {
		A_SEQUENCE_OF(struct spatem_ts_Reg_MovementState) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} spatem_ts_MovementState_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_MovementState;
extern asn_SEQUENCE_specifics_t asn_SPC_spatem_ts_MovementState_specs_1;
extern asn_TYPE_member_t asn_MBR_spatem_ts_MovementState_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_ts_coding/spatem_ts_ManeuverAssistList.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_RegionalExtension.h"

#endif	/* _spatem_ts_MovementState_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
