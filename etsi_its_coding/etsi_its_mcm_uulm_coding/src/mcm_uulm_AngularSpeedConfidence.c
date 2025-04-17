/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#include "etsi_its_mcm_uulm_coding/mcm_uulm_AngularSpeedConfidence.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_mcm_uulm_AngularSpeedConfidence_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_mcm_uulm_AngularSpeedConfidence_value2enum_1[] = {
	{ 0,	9,	"degSec-01" },
	{ 1,	9,	"degSec-02" },
	{ 2,	9,	"degSec-05" },
	{ 3,	9,	"degSec-10" },
	{ 4,	9,	"degSec-20" },
	{ 5,	9,	"degSec-50" },
	{ 6,	10,	"outOfRange" },
	{ 7,	11,	"unavailable" }
};
static const unsigned int asn_MAP_mcm_uulm_AngularSpeedConfidence_enum2value_1[] = {
	0,	/* degSec-01(0) */
	1,	/* degSec-02(1) */
	2,	/* degSec-05(2) */
	3,	/* degSec-10(3) */
	4,	/* degSec-20(4) */
	5,	/* degSec-50(5) */
	6,	/* outOfRange(6) */
	7	/* unavailable(7) */
};
const asn_INTEGER_specifics_t asn_SPC_mcm_uulm_AngularSpeedConfidence_specs_1 = {
	asn_MAP_mcm_uulm_AngularSpeedConfidence_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_mcm_uulm_AngularSpeedConfidence_enum2value_1,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_mcm_uulm_AngularSpeedConfidence = {
	"AngularSpeedConfidence",
	"AngularSpeedConfidence",
	&asn_OP_NativeEnumerated,
	asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1,
	sizeof(asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1)
		/sizeof(asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1[0]), /* 1 */
	asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1,	/* Same as above */
	sizeof(asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1)
		/sizeof(asn_DEF_mcm_uulm_AngularSpeedConfidence_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_mcm_uulm_AngularSpeedConfidence_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_mcm_uulm_AngularSpeedConfidence_specs_1	/* Additional specs */
};

