/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_LowerTriangularPositiveSemidefiniteMatrices_H_
#define	_cpm_LowerTriangularPositiveSemidefiniteMatrices_H_


#include <etsi_its_cpm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_cpm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cpm_LowerTriangularPositiveSemidefiniteMatrix;

/* cpm_LowerTriangularPositiveSemidefiniteMatrices */
typedef struct cpm_LowerTriangularPositiveSemidefiniteMatrices {
	A_SEQUENCE_OF(struct cpm_LowerTriangularPositiveSemidefiniteMatrix) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_LowerTriangularPositiveSemidefiniteMatrices_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_LowerTriangularPositiveSemidefiniteMatrices;
extern asn_SET_OF_specifics_t asn_SPC_cpm_LowerTriangularPositiveSemidefiniteMatrices_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_LowerTriangularPositiveSemidefiniteMatrices_1[1];
extern asn_per_constraints_t asn_PER_type_cpm_LowerTriangularPositiveSemidefiniteMatrices_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cpm_coding/cpm_LowerTriangularPositiveSemidefiniteMatrix.h"

#endif	/* _cpm_LowerTriangularPositiveSemidefiniteMatrices_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
