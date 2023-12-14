/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_spatem_coding/BasicVehicleRole.h"
#include "etsi_its_spatem_coding/RequestSubRole.h"
#include "etsi_its_spatem_coding/RequestImportanceLevel.h"
#include "etsi_its_spatem_coding/Iso3833VehicleType.h"
#include "etsi_its_spatem_coding/VehicleType.h"
#include <etsi_its_spatem_coding/constr_SEQUENCE.h>
#ifndef	_RequestorType_H_
#define	_RequestorType_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Reg_RequestorType;

/* RequestorType */
typedef struct RequestorType {
	BasicVehicleRole_t	 role;
	RequestSubRole_t	*subrole;	/* OPTIONAL */
	RequestImportanceLevel_t	*request;	/* OPTIONAL */
	Iso3833VehicleType_t	*iso3883;	/* OPTIONAL */
	VehicleType_t	*hpmsType;	/* OPTIONAL */
	struct Reg_RequestorType	*regional;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RequestorType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RequestorType;
extern asn_SEQUENCE_specifics_t asn_SPC_RequestorType_specs_1;
extern asn_TYPE_member_t asn_MBR_RequestorType_1[6];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_coding/RegionalExtension.h"

#endif	/* _RequestorType_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>