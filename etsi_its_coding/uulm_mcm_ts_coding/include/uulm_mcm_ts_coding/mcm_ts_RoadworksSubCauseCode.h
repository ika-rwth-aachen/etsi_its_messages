/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_RoadworksSubCauseCode_H_
#define	_mcm_ts_RoadworksSubCauseCode_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_RoadworksSubCauseCode {
	mcm_ts_RoadworksSubCauseCode_unavailable	= 0,
	mcm_ts_RoadworksSubCauseCode_majorRoadworks	= 1,
	mcm_ts_RoadworksSubCauseCode_roadMarkingWork	= 2,
	mcm_ts_RoadworksSubCauseCode_slowMovingRoadMaintenance	= 3,
	mcm_ts_RoadworksSubCauseCode_shortTermStationaryRoadworks	= 4,
	mcm_ts_RoadworksSubCauseCode_streetCleaning	= 5,
	mcm_ts_RoadworksSubCauseCode_winterService	= 6
} e_mcm_ts_RoadworksSubCauseCode;

/* mcm_ts_RoadworksSubCauseCode */
typedef long	 mcm_ts_RoadworksSubCauseCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_ts_RoadworksSubCauseCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_RoadworksSubCauseCode;
asn_struct_free_f mcm_ts_RoadworksSubCauseCode_free;
asn_struct_print_f mcm_ts_RoadworksSubCauseCode_print;
asn_constr_check_f mcm_ts_RoadworksSubCauseCode_constraint;
per_type_decoder_f mcm_ts_RoadworksSubCauseCode_decode_uper;
per_type_encoder_f mcm_ts_RoadworksSubCauseCode_encode_uper;
per_type_decoder_f mcm_ts_RoadworksSubCauseCode_decode_aper;
per_type_encoder_f mcm_ts_RoadworksSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_RoadworksSubCauseCode_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
