/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#include "etsi_its_mapem_ts_coding/mapem_ts_CurvatureCalculationMode.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_mapem_ts_CurvatureCalculationMode_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  0,  2 }	/* (0..2,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_mapem_ts_CurvatureCalculationMode_value2enum_1[] = {
	{ 0,	11,	"yawRateUsed" },
	{ 1,	14,	"yawRateNotUsed" },
	{ 2,	11,	"unavailable" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_mapem_ts_CurvatureCalculationMode_enum2value_1[] = {
	2,	/* unavailable(2) */
	1,	/* yawRateNotUsed(1) */
	0	/* yawRateUsed(0) */
	/* This list is extensible */
};
static const asn_INTEGER_specifics_t asn_SPC_mapem_ts_CurvatureCalculationMode_specs_1 = {
	asn_MAP_mapem_ts_CurvatureCalculationMode_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_mapem_ts_CurvatureCalculationMode_enum2value_1,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	4,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_mapem_ts_CurvatureCalculationMode = {
	"CurvatureCalculationMode",
	"CurvatureCalculationMode",
	&asn_OP_NativeEnumerated,
	asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1,
	sizeof(asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1)
		/sizeof(asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1[0]), /* 1 */
	asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1,	/* Same as above */
	sizeof(asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1)
		/sizeof(asn_DEF_mapem_ts_CurvatureCalculationMode_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_mapem_ts_CurvatureCalculationMode_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_mapem_ts_CurvatureCalculationMode_specs_1	/* Additional specs */
};

