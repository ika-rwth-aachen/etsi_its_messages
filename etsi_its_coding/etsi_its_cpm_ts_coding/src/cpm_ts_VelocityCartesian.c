/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#include "etsi_its_cpm_ts_coding/cpm_ts_VelocityCartesian.h"

asn_TYPE_member_t asn_MBR_cpm_ts_VelocityCartesian_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct cpm_ts_VelocityCartesian, xVelocity),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_cpm_ts_VelocityComponent,
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
		"xVelocity"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct cpm_ts_VelocityCartesian, yVelocity),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_cpm_ts_VelocityComponent,
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
		"yVelocity"
		},
	{ ATF_POINTER, 1, offsetof(struct cpm_ts_VelocityCartesian, zVelocity),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_cpm_ts_VelocityComponent,
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
		"zVelocity"
		},
};
static const int asn_MAP_cpm_ts_VelocityCartesian_oms_1[] = { 2 };
static const ber_tlv_tag_t asn_DEF_cpm_ts_VelocityCartesian_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_cpm_ts_VelocityCartesian_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* xVelocity */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* yVelocity */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* zVelocity */
};
asn_SEQUENCE_specifics_t asn_SPC_cpm_ts_VelocityCartesian_specs_1 = {
	sizeof(struct cpm_ts_VelocityCartesian),
	offsetof(struct cpm_ts_VelocityCartesian, _asn_ctx),
	asn_MAP_cpm_ts_VelocityCartesian_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_cpm_ts_VelocityCartesian_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_cpm_ts_VelocityCartesian = {
	"VelocityCartesian",
	"VelocityCartesian",
	&asn_OP_SEQUENCE,
	asn_DEF_cpm_ts_VelocityCartesian_tags_1,
	sizeof(asn_DEF_cpm_ts_VelocityCartesian_tags_1)
		/sizeof(asn_DEF_cpm_ts_VelocityCartesian_tags_1[0]), /* 1 */
	asn_DEF_cpm_ts_VelocityCartesian_tags_1,	/* Same as above */
	sizeof(asn_DEF_cpm_ts_VelocityCartesian_tags_1)
		/sizeof(asn_DEF_cpm_ts_VelocityCartesian_tags_1[0]), /* 1 */
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
		SEQUENCE_constraint
	},
	asn_MBR_cpm_ts_VelocityCartesian_1,
	3,	/* Elements count */
	&asn_SPC_cpm_ts_VelocityCartesian_specs_1	/* Additional specs */
};

