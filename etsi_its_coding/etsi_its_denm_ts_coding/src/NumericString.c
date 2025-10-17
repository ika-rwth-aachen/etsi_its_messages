/*-
 * Copyright (c) 2003, 2006 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <etsi_its_denm_ts_coding/asn_internal.h>
#include <etsi_its_denm_ts_coding/NumericString.h>

/*
 * NumericString basic type description.
 */
static const ber_tlv_tag_t asn_DEF_NumericString_tags[] = {
    (ASN_TAG_CLASS_UNIVERSAL | (18 << 2)),  /* [UNIVERSAL 18] IMPLICIT ...*/
    (ASN_TAG_CLASS_UNIVERSAL | (4 << 2))    /* ... OCTET STRING */
};
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static int asn_DEF_NumericString_v2c(unsigned int value) {
    switch(value) {
    case 0x20: return 0;
    case 0x30: case 0x31: case 0x32: case 0x33: case 0x34:
    case 0x35: case 0x36: case 0x37: case 0x38: case 0x39:
        return value - (0x30 - 1);
    }
    return -1;
}
static int asn_DEF_NumericString_c2v(unsigned int code) {
    if(code > 0) {
        if(code <= 10)
            return code + (0x30 - 1);
        else
            return -1;
    } else {
        return 0x20;
    }
}
static asn_per_constraints_t asn_DEF_NumericString_per_constraints = {
    { APC_CONSTRAINED, 4, 4, 0x20, 0x39 },   /* Value */
    { APC_SEMI_CONSTRAINED, -1, -1, 0, 0 },  /* Size */
    asn_DEF_NumericString_v2c,
    asn_DEF_NumericString_c2v
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
asn_TYPE_operation_t asn_OP_NumericString = {
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
#endif  /* !defined(ASN_DISABLE_XER_SUPPORT) */
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
asn_TYPE_descriptor_t asn_DEF_NumericString = {
    "NumericString",
    "NumericString",
    &asn_OP_NumericString,
    asn_DEF_NumericString_tags,
    sizeof(asn_DEF_NumericString_tags)
      / sizeof(asn_DEF_NumericString_tags[0]) - 1,
    asn_DEF_NumericString_tags,
    sizeof(asn_DEF_NumericString_tags)
      / sizeof(asn_DEF_NumericString_tags[0]),
    {
#if !defined(ASN_DISABLE_OER_SUPPORT)
        0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
        &asn_DEF_NumericString_per_constraints,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
        0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
        NumericString_constraint
    },
    0, 0,  /* No members */
    0  /* No specifics */
};

int
NumericString_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
                         asn_app_constraint_failed_f *ctfailcb, void *app_key) {
    const NumericString_t *st = (const NumericString_t *)sptr;

	if(st && st->buf) {
		uint8_t *buf = st->buf;
		uint8_t *end = buf + st->size;

		/*
		 * Check the alphabet of the NumericString.
		 * ASN.1:1984 (X.409)
		 */
		for(; buf < end; buf++) {
			switch(*buf) {
			case 0x20:
			case 0x30: case 0x31: case 0x32: case 0x33: case 0x34:
			case 0x35: case 0x36: case 0x37: case 0x38: case 0x39:
				continue;
			}
			ASN__CTFAIL(app_key, td, sptr,
				"%s: value byte %ld (%d) "
				"not in NumericString alphabet (%s:%d)",
				td->name,
				(long)((buf - st->buf) + 1),
				*buf,
				__FILE__, __LINE__);
			return -1;
		}
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}

	return 0;
}
