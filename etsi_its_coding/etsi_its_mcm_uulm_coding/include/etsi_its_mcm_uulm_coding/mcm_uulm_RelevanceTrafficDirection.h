/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_RelevanceTrafficDirection_H_
#define	_mcm_uulm_RelevanceTrafficDirection_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_RelevanceTrafficDirection {
	mcm_uulm_RelevanceTrafficDirection_allTrafficDirections	= 0,
	mcm_uulm_RelevanceTrafficDirection_upstreamTraffic	= 1,
	mcm_uulm_RelevanceTrafficDirection_downstreamTraffic	= 2,
	mcm_uulm_RelevanceTrafficDirection_oppositeTraffic	= 3
} e_mcm_uulm_RelevanceTrafficDirection;

/* mcm_uulm_RelevanceTrafficDirection */
typedef long	 mcm_uulm_RelevanceTrafficDirection_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_RelevanceTrafficDirection;
asn_struct_free_f mcm_uulm_RelevanceTrafficDirection_free;
asn_struct_print_f mcm_uulm_RelevanceTrafficDirection_print;
asn_constr_check_f mcm_uulm_RelevanceTrafficDirection_constraint;
jer_type_encoder_f mcm_uulm_RelevanceTrafficDirection_encode_jer;
per_type_decoder_f mcm_uulm_RelevanceTrafficDirection_decode_uper;
per_type_encoder_f mcm_uulm_RelevanceTrafficDirection_encode_uper;
per_type_decoder_f mcm_uulm_RelevanceTrafficDirection_decode_aper;
per_type_encoder_f mcm_uulm_RelevanceTrafficDirection_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_RelevanceTrafficDirection_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
