/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_ClusterBreakupReason_H_
#define	_mcm_uulm_ClusterBreakupReason_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_ClusterBreakupReason {
	mcm_uulm_ClusterBreakupReason_notProvided	= 0,
	mcm_uulm_ClusterBreakupReason_clusteringPurposeCompleted	= 1,
	mcm_uulm_ClusterBreakupReason_leaderMovedOutOfClusterBoundingBox	= 2,
	mcm_uulm_ClusterBreakupReason_joiningAnotherCluster	= 3,
	mcm_uulm_ClusterBreakupReason_enteringLowRiskAreaBasedOnMaps	= 4,
	mcm_uulm_ClusterBreakupReason_receptionOfCpmContainingCluster	= 5
} e_mcm_uulm_ClusterBreakupReason;

/* mcm_uulm_ClusterBreakupReason */
typedef long	 mcm_uulm_ClusterBreakupReason_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_uulm_ClusterBreakupReason_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_ClusterBreakupReason;
asn_struct_free_f mcm_uulm_ClusterBreakupReason_free;
asn_struct_print_f mcm_uulm_ClusterBreakupReason_print;
asn_constr_check_f mcm_uulm_ClusterBreakupReason_constraint;
jer_type_encoder_f mcm_uulm_ClusterBreakupReason_encode_jer;
per_type_decoder_f mcm_uulm_ClusterBreakupReason_decode_uper;
per_type_encoder_f mcm_uulm_ClusterBreakupReason_encode_uper;
per_type_decoder_f mcm_uulm_ClusterBreakupReason_decode_aper;
per_type_encoder_f mcm_uulm_ClusterBreakupReason_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_ClusterBreakupReason_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
