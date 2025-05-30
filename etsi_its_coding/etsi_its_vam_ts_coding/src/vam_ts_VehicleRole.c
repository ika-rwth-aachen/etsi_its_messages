/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#include "etsi_its_vam_ts_coding/vam_ts_VehicleRole.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_vam_ts_VehicleRole_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_vam_ts_VehicleRole_value2enum_1[] = {
	{ 0,	7,	"default" },
	{ 1,	15,	"publicTransport" },
	{ 2,	16,	"specialTransport" },
	{ 3,	14,	"dangerousGoods" },
	{ 4,	8,	"roadWork" },
	{ 5,	6,	"rescue" },
	{ 6,	9,	"emergency" },
	{ 7,	9,	"safetyCar" },
	{ 8,	11,	"agriculture" },
	{ 9,	10,	"commercial" },
	{ 10,	8,	"military" },
	{ 11,	12,	"roadOperator" },
	{ 12,	4,	"taxi" },
	{ 13,	9,	"reserved1" },
	{ 14,	9,	"reserved2" },
	{ 15,	9,	"reserved3" }
};
static const unsigned int asn_MAP_vam_ts_VehicleRole_enum2value_1[] = {
	8,	/* agriculture(8) */
	9,	/* commercial(9) */
	3,	/* dangerousGoods(3) */
	0,	/* default(0) */
	6,	/* emergency(6) */
	10,	/* military(10) */
	1,	/* publicTransport(1) */
	5,	/* rescue(5) */
	13,	/* reserved1(13) */
	14,	/* reserved2(14) */
	15,	/* reserved3(15) */
	11,	/* roadOperator(11) */
	4,	/* roadWork(4) */
	7,	/* safetyCar(7) */
	2,	/* specialTransport(2) */
	12	/* taxi(12) */
};
static const asn_INTEGER_specifics_t asn_SPC_vam_ts_VehicleRole_specs_1 = {
	asn_MAP_vam_ts_VehicleRole_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_vam_ts_VehicleRole_enum2value_1,	/* N => "tag"; sorted by N */
	16,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_vam_ts_VehicleRole_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_vam_ts_VehicleRole = {
	"VehicleRole",
	"VehicleRole",
	&asn_OP_NativeEnumerated,
	asn_DEF_vam_ts_VehicleRole_tags_1,
	sizeof(asn_DEF_vam_ts_VehicleRole_tags_1)
		/sizeof(asn_DEF_vam_ts_VehicleRole_tags_1[0]), /* 1 */
	asn_DEF_vam_ts_VehicleRole_tags_1,	/* Same as above */
	sizeof(asn_DEF_vam_ts_VehicleRole_tags_1)
		/sizeof(asn_DEF_vam_ts_VehicleRole_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_vam_ts_VehicleRole_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_vam_ts_VehicleRole_specs_1	/* Additional specs */
};

