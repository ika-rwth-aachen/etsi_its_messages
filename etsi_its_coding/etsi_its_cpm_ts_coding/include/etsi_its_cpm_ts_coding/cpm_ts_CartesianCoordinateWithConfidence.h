/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cpm_ts_CartesianCoordinateWithConfidence_H_
#define	_cpm_ts_CartesianCoordinateWithConfidence_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cpm_ts_coding/cpm_ts_CartesianCoordinateLarge.h"
#include "etsi_its_cpm_ts_coding/cpm_ts_CoordinateConfidence.h"
#include <etsi_its_cpm_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* cpm_ts_CartesianCoordinateWithConfidence */
typedef struct cpm_ts_CartesianCoordinateWithConfidence {
	cpm_ts_CartesianCoordinateLarge_t	 value;
	cpm_ts_CoordinateConfidence_t	 confidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_ts_CartesianCoordinateWithConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_CartesianCoordinateWithConfidence;
extern asn_SEQUENCE_specifics_t asn_SPC_cpm_ts_CartesianCoordinateWithConfidence_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_ts_CartesianCoordinateWithConfidence_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_ts_CartesianCoordinateWithConfidence_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>
