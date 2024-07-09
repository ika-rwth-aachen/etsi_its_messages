/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-SensorInformationContainer"
 * 	found in "/input/CPM-SensorInformationContainer.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#include "etsi_its_cpm_ts_coding/cpm_ts_SensorInformationContainer.h"

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_cpm_ts_SensorInformationContainer_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  7,  7,  1,  128 }	/* (SIZE(1..128,...)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
asn_TYPE_member_t asn_MBR_cpm_ts_SensorInformationContainer_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_cpm_ts_SensorInformation,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_cpm_ts_SensorInformationContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_cpm_ts_SensorInformationContainer_specs_1 = {
	sizeof(struct cpm_ts_SensorInformationContainer),
	offsetof(struct cpm_ts_SensorInformationContainer, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_cpm_ts_SensorInformationContainer = {
	"SensorInformationContainer",
	"SensorInformationContainer",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_cpm_ts_SensorInformationContainer_tags_1,
	sizeof(asn_DEF_cpm_ts_SensorInformationContainer_tags_1)
		/sizeof(asn_DEF_cpm_ts_SensorInformationContainer_tags_1[0]), /* 1 */
	asn_DEF_cpm_ts_SensorInformationContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_cpm_ts_SensorInformationContainer_tags_1)
		/sizeof(asn_DEF_cpm_ts_SensorInformationContainer_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_cpm_ts_SensorInformationContainer_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_cpm_ts_SensorInformationContainer_1,
	1,	/* Single element */
	&asn_SPC_cpm_ts_SensorInformationContainer_specs_1	/* Additional specs */
};
