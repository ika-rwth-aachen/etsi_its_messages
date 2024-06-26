/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_DangerousGoodsBasic_H_
#define	_cpm_DangerousGoodsBasic_H_


#include <etsi_its_cpm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cpm_DangerousGoodsBasic {
	cpm_DangerousGoodsBasic_explosives1	= 0,
	cpm_DangerousGoodsBasic_explosives2	= 1,
	cpm_DangerousGoodsBasic_explosives3	= 2,
	cpm_DangerousGoodsBasic_explosives4	= 3,
	cpm_DangerousGoodsBasic_explosives5	= 4,
	cpm_DangerousGoodsBasic_explosives6	= 5,
	cpm_DangerousGoodsBasic_flammableGases	= 6,
	cpm_DangerousGoodsBasic_nonFlammableGases	= 7,
	cpm_DangerousGoodsBasic_toxicGases	= 8,
	cpm_DangerousGoodsBasic_flammableLiquids	= 9,
	cpm_DangerousGoodsBasic_flammableSolids	= 10,
	cpm_DangerousGoodsBasic_substancesLiableToSpontaneousCombustion	= 11,
	cpm_DangerousGoodsBasic_substancesEmittingFlammableGasesUponContactWithWater	= 12,
	cpm_DangerousGoodsBasic_oxidizingSubstances	= 13,
	cpm_DangerousGoodsBasic_organicPeroxides	= 14,
	cpm_DangerousGoodsBasic_toxicSubstances	= 15,
	cpm_DangerousGoodsBasic_infectiousSubstances	= 16,
	cpm_DangerousGoodsBasic_radioactiveMaterial	= 17,
	cpm_DangerousGoodsBasic_corrosiveSubstances	= 18,
	cpm_DangerousGoodsBasic_miscellaneousDangerousSubstances	= 19
} e_cpm_DangerousGoodsBasic;

/* cpm_DangerousGoodsBasic */
typedef long	 cpm_DangerousGoodsBasic_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cpm_DangerousGoodsBasic_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cpm_DangerousGoodsBasic;
extern const asn_INTEGER_specifics_t asn_SPC_cpm_DangerousGoodsBasic_specs_1;
asn_struct_free_f cpm_DangerousGoodsBasic_free;
asn_struct_print_f cpm_DangerousGoodsBasic_print;
asn_constr_check_f cpm_DangerousGoodsBasic_constraint;
per_type_decoder_f cpm_DangerousGoodsBasic_decode_uper;
per_type_encoder_f cpm_DangerousGoodsBasic_encode_uper;
per_type_decoder_f cpm_DangerousGoodsBasic_decode_aper;
per_type_encoder_f cpm_DangerousGoodsBasic_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_DangerousGoodsBasic_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
