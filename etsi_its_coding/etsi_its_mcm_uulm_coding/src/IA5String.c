/*-
 * Copyright (c) 2003 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
#include <etsi_its_mcm_uulm_coding/IA5String.h>

/*
 * IA5String basic type description.
 */
static const ber_tlv_tag_t asn_DEF_IA5String_tags[] = {
    (ASN_TAG_CLASS_UNIVERSAL | (22 << 2)),  /* [UNIVERSAL 22] IMPLICIT ...*/
    (ASN_TAG_CLASS_UNIVERSAL | (4 << 2))    /* ... OCTET STRING */
};
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_DEF_IA5String_per_constraints = {
    { APC_CONSTRAINED, 7, 7, 0, 0x7f },	/* Value */
    { APC_SEMI_CONSTRAINED, -1, -1, 0, 0 },	/* Size */
    0, 0
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
asn_TYPE_operation_t asn_OP_IA5String = {
    OCTET_STRING_free,
#if !defined(ASN_DISABLE_PRINT_SUPPORT)
    OCTET_STRING_print_utf8,  /* ASCII subset */
#else
    0,
#endif  /* !defined(ASN_DISABLE_PRINT_SUPPORT) */
    OCTET_STRING_compare,
    OCTET_STRING_copy,
#if !defined(ASN_DISABLE_BER_SUPPORT)
    OCTET_STRING_decode_ber,  /* Implemented in terms of OCTET STRING */
    OCTET_STRING_encode_der,
#else
    0,
    0,
#endif  /* !defined(ASN_DISABLE_BER_SUPPORT) */
#if !defined(ASN_DISABLE_XER_SUPPORT)
    OCTET_STRING_decode_xer_utf8,
    OCTET_STRING_encode_xer_utf8,
#else
    0,
    0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
    OCTET_STRING_decode_jer_utf8,
    OCTET_STRING_encode_jer_utf8,
#else
    0,
    0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
#if !defined(ASN_DISABLE_OER_SUPPORT)
    OCTET_STRING_decode_oer,
    OCTET_STRING_encode_oer,
#else
    0,
    0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT)
    OCTET_STRING_decode_uper,
    OCTET_STRING_encode_uper,
#else
    0,
    0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) */
#if !defined(ASN_DISABLE_APER_SUPPORT)
    OCTET_STRING_decode_aper,
    OCTET_STRING_encode_aper,
#else
    0,
    0,
#endif  /* !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_RFILL_SUPPORT)
    OCTET_STRING_random_fill,
#else
    0,
#endif  /* !defined(ASN_DISABLE_RFILL_SUPPORT) */
    0  /* Use generic outmost tag fetcher */
};
asn_TYPE_descriptor_t asn_DEF_IA5String = {
    "IA5String",
    "IA5String",
    &asn_OP_IA5String,
    asn_DEF_IA5String_tags,
    sizeof(asn_DEF_IA5String_tags)
      / sizeof(asn_DEF_IA5String_tags[0]) - 1,
    asn_DEF_IA5String_tags,
    sizeof(asn_DEF_IA5String_tags)
      / sizeof(asn_DEF_IA5String_tags[0]),
    {
#if !defined(ASN_DISABLE_OER_SUPPORT)
        0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
        &asn_DEF_IA5String_per_constraints,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
        0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
        IA5String_constraint
    },
    0, 0,  /* No members */
    0  /* No specifics */
};

int
IA5String_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
                     asn_app_constraint_failed_f *ctfailcb, void *app_key) {
    const IA5String_t *st = (const IA5String_t *)sptr;

	if(st && st->buf) {
		uint8_t *buf = st->buf;
		uint8_t *end = buf + st->size;
		/*
		 * IA5String is generally equivalent to 7bit ASCII.
		 * ISO/ITU-T T.50, 1963.
		 */
		for(; buf < end; buf++) {
			if(*buf > 0x7F) {
				ASN__CTFAIL(app_key, td, sptr,
					"%s: value byte %ld out of range: "
					"%d > 127 (%s:%d)",
					td->name,
					(long)((buf - st->buf) + 1),
					*buf,
					__FILE__, __LINE__);
				return -1;
			}
		}
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}

	return 0;
}

