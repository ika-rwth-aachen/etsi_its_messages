/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_CartesianPosition3dWithConfidence_H_
#define	_cpm_CartesianPosition3dWithConfidence_H_


#include <etsi_its_cpm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cpm_coding/cpm_CartesianCoordinateWithConfidence.h"
#include <etsi_its_cpm_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cpm_CartesianCoordinateWithConfidence;

/* cpm_CartesianPosition3dWithConfidence */
typedef struct cpm_CartesianPosition3dWithConfidence {
	cpm_CartesianCoordinateWithConfidence_t	 xCoordinate;
	cpm_CartesianCoordinateWithConfidence_t	 yCoordinate;
	struct cpm_CartesianCoordinateWithConfidence	*zCoordinate;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_CartesianPosition3dWithConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_CartesianPosition3dWithConfidence;
extern asn_SEQUENCE_specifics_t asn_SPC_cpm_CartesianPosition3dWithConfidence_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_CartesianPosition3dWithConfidence_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cpm_coding/cpm_CartesianCoordinateWithConfidence.h"

#endif	/* _cpm_CartesianPosition3dWithConfidence_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
