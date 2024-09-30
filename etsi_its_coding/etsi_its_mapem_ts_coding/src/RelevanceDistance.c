/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_mapem_ts_coding/RelevanceDistance.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_RelevanceDistance_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_RelevanceDistance_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_RelevanceDistance_value2enum_1[] = {
	{ 0,	11,	"lessThan50m" },
	{ 1,	12,	"lessThan100m" },
	{ 2,	12,	"lessThan200m" },
	{ 3,	12,	"lessThan500m" },
	{ 4,	13,	"lessThan1000m" },
	{ 5,	11,	"lessThan5km" },
	{ 6,	12,	"lessThan10km" },
	{ 7,	8,	"over10km" }
};
static const unsigned int asn_MAP_RelevanceDistance_enum2value_1[] = {
	4,	/* lessThan1000m(4) */
	1,	/* lessThan100m(1) */
	6,	/* lessThan10km(6) */
	2,	/* lessThan200m(2) */
	3,	/* lessThan500m(3) */
	0,	/* lessThan50m(0) */
	5,	/* lessThan5km(5) */
	7	/* over10km(7) */
};
static const asn_INTEGER_specifics_t asn_SPC_RelevanceDistance_specs_1 = {
	asn_MAP_RelevanceDistance_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_RelevanceDistance_enum2value_1,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_RelevanceDistance_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_RelevanceDistance = {
	"RelevanceDistance",
	"RelevanceDistance",
	&asn_OP_NativeEnumerated,
	asn_DEF_RelevanceDistance_tags_1,
	sizeof(asn_DEF_RelevanceDistance_tags_1)
		/sizeof(asn_DEF_RelevanceDistance_tags_1[0]), /* 1 */
	asn_DEF_RelevanceDistance_tags_1,	/* Same as above */
	sizeof(asn_DEF_RelevanceDistance_tags_1)
		/sizeof(asn_DEF_RelevanceDistance_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_RelevanceDistance_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_RelevanceDistance_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_RelevanceDistance_specs_1	/* Additional specs */
};

