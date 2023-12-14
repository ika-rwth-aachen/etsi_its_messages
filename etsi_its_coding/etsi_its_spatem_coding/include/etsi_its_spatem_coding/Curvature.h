/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_spatem_coding/CurvatureValue.h"
#include "etsi_its_spatem_coding/CurvatureConfidence.h"
#include <etsi_its_spatem_coding/constr_SEQUENCE.h>
#ifndef	_Curvature_H_
#define	_Curvature_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Curvature */
typedef struct Curvature {
	CurvatureValue_t	 curvatureValue;
	CurvatureConfidence_t	 curvatureConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Curvature_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Curvature;

#ifdef __cplusplus
}
#endif

#endif	/* _Curvature_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
