/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_PosConfidenceEllipse_H_
#define	_spatem_ts_PosConfidenceEllipse_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_spatem_ts_coding/spatem_ts_SemiAxisLength.h"
#include "etsi_its_spatem_ts_coding/spatem_ts_HeadingValue.h"
#include <etsi_its_spatem_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* spatem_ts_PosConfidenceEllipse */
typedef struct spatem_ts_PosConfidenceEllipse {
	spatem_ts_SemiAxisLength_t	 semiMajorConfidence;
	spatem_ts_SemiAxisLength_t	 semiMinorConfidence;
	spatem_ts_HeadingValue_t	 semiMajorOrientation;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} spatem_ts_PosConfidenceEllipse_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_PosConfidenceEllipse;
extern asn_SEQUENCE_specifics_t asn_SPC_spatem_ts_PosConfidenceEllipse_specs_1;
extern asn_TYPE_member_t asn_MBR_spatem_ts_PosConfidenceEllipse_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_PosConfidenceEllipse_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
