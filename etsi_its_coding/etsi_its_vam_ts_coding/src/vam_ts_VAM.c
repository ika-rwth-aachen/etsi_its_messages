/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "VAM-PDU-Descriptions"
 * 	found in "/input/VAM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#include "etsi_its_vam_ts_coding/vam_ts_VAM.h"

static asn_TYPE_member_t asn_MBR_vam_ts_VAM_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct vam_ts_VAM, header),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_vam_ts_ItsPduHeaderVam,
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
		"header"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct vam_ts_VAM, vam),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_vam_ts_VruAwareness,
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
		"vam"
		},
};
static const ber_tlv_tag_t asn_DEF_vam_ts_VAM_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_vam_ts_VAM_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* header */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* vam */
};
static asn_SEQUENCE_specifics_t asn_SPC_vam_ts_VAM_specs_1 = {
	sizeof(struct vam_ts_VAM),
	offsetof(struct vam_ts_VAM, _asn_ctx),
	asn_MAP_vam_ts_VAM_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_vam_ts_VAM = {
	"VAM",
	"VAM",
	&asn_OP_SEQUENCE,
	asn_DEF_vam_ts_VAM_tags_1,
	sizeof(asn_DEF_vam_ts_VAM_tags_1)
		/sizeof(asn_DEF_vam_ts_VAM_tags_1[0]), /* 1 */
	asn_DEF_vam_ts_VAM_tags_1,	/* Same as above */
	sizeof(asn_DEF_vam_ts_VAM_tags_1)
		/sizeof(asn_DEF_vam_ts_VAM_tags_1[0]), /* 1 */
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
	asn_MBR_vam_ts_VAM_1,
	2,	/* Elements count */
	&asn_SPC_vam_ts_VAM_specs_1	/* Additional specs */
};

