/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_mapem_ts_coding/TransitVehicleOccupancy.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_TransitVehicleOccupancy_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_TransitVehicleOccupancy_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_TransitVehicleOccupancy_value2enum_1[] = {
	{ 0,	16,	"occupancyUnknown" },
	{ 1,	14,	"occupancyEmpty" },
	{ 2,	16,	"occupancyVeryLow" },
	{ 3,	12,	"occupancyLow" },
	{ 4,	12,	"occupancyMed" },
	{ 5,	13,	"occupancyHigh" },
	{ 6,	19,	"occupancyNearlyFull" },
	{ 7,	13,	"occupancyFull" }
};
static const unsigned int asn_MAP_TransitVehicleOccupancy_enum2value_1[] = {
	1,	/* occupancyEmpty(1) */
	7,	/* occupancyFull(7) */
	5,	/* occupancyHigh(5) */
	3,	/* occupancyLow(3) */
	4,	/* occupancyMed(4) */
	6,	/* occupancyNearlyFull(6) */
	0,	/* occupancyUnknown(0) */
	2	/* occupancyVeryLow(2) */
};
const asn_INTEGER_specifics_t asn_SPC_TransitVehicleOccupancy_specs_1 = {
	asn_MAP_TransitVehicleOccupancy_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_TransitVehicleOccupancy_enum2value_1,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_TransitVehicleOccupancy_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_TransitVehicleOccupancy = {
	"TransitVehicleOccupancy",
	"TransitVehicleOccupancy",
	&asn_OP_NativeEnumerated,
	asn_DEF_TransitVehicleOccupancy_tags_1,
	sizeof(asn_DEF_TransitVehicleOccupancy_tags_1)
		/sizeof(asn_DEF_TransitVehicleOccupancy_tags_1[0]), /* 1 */
	asn_DEF_TransitVehicleOccupancy_tags_1,	/* Same as above */
	sizeof(asn_DEF_TransitVehicleOccupancy_tags_1)
		/sizeof(asn_DEF_TransitVehicleOccupancy_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_TransitVehicleOccupancy_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_TransitVehicleOccupancy_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_TransitVehicleOccupancy_specs_1	/* Additional specs */
};

