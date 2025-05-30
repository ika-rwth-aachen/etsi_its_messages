/*
 * Copyright (c) 2017 Lev Walkin <vlm@lionet.info>.
 * All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <etsi_its_vam_ts_coding/asn_internal.h>
#include <etsi_its_vam_ts_coding/constr_SET_OF.h>

asn_dec_rval_t
SET_OF_decode_uper(const asn_codec_ctx_t *opt_codec_ctx,
                   const asn_TYPE_descriptor_t *td,
                   const asn_per_constraints_t *constraints, void **sptr,
                   asn_per_data_t *pd) {
    asn_dec_rval_t rv = {RC_OK, 0};
    const asn_SET_OF_specifics_t *specs = (const asn_SET_OF_specifics_t *)td->specifics;
    const asn_TYPE_member_t *elm = td->elements;  /* Single one */
    void *st = *sptr;
    asn_anonymous_set_ *list;
    const asn_per_constraint_t *ct;
    int repeat = 0;
    ssize_t nelems;

    if(ASN__STACK_OVERFLOW_CHECK(opt_codec_ctx))
        ASN__DECODE_FAILED;

    /*
     * Create the target structure if it is not present already.
     */
    if(!st) {
        st = *sptr = CALLOC(1, specs->struct_size);
        if(!st) ASN__DECODE_FAILED;
    }
    list = _A_SET_FROM_VOID(st);

    /* Figure out which constraints to use */
    if(constraints) ct = &constraints->size;
    else if(td->encoding_constraints.per_constraints)
        ct = &td->encoding_constraints.per_constraints->size;
    else ct = 0;

    if(ct && ct->flags & APC_EXTENSIBLE) {
        int value = per_get_few_bits(pd, 1);
        if(value < 0) ASN__DECODE_STARVED;
        if(value) ct = 0;  /* Not restricted! */
    }

    if(ct && ct->effective_bits >= 0) {
        /* X.691, #19.5: No length determinant */
        nelems = per_get_few_bits(pd, ct->effective_bits);
        ASN_DEBUG("Preparing to fetch %ld+%"ASN_PRIdMAX" elements from %s",
                  (long)nelems, ct->lower_bound, td->name);
        if(nelems < 0)  ASN__DECODE_STARVED;
        nelems += ct->lower_bound;
	/* check if nelem is in root, when it is not extensible */
        if (nelems > ct->upper_bound && !(ct->flags & APC_EXTENSIBLE)) {
            ASN__DECODE_FAILED;
        }
    } else {
        nelems = -1;
    }

    do {
        int i;
        if(nelems < 0) {
            nelems = uper_get_length(pd, -1, 0, &repeat);
            ASN_DEBUG("Got to decode %" ASN_PRI_SSIZE " elements (eff %d)",
                      nelems, (int)(ct ? ct->effective_bits : -1));
            if(nelems < 0) ASN__DECODE_STARVED;
        }

        for(i = 0; i < nelems; i++) {
            void *ptr = 0;
            ASN_DEBUG("SET OF %s decoding", elm->type->name);
            rv = elm->type->op->uper_decoder(opt_codec_ctx, elm->type,
                                             elm->encoding_constraints.per_constraints,
                                             &ptr, pd);
            ASN_DEBUG("%s SET OF %s decoded %d, %p",
                      td->name, elm->type->name, rv.code, ptr);
            if(rv.code == RC_OK) {
                if(ASN_SET_ADD(list, ptr) == 0) {
                    if(rv.consumed == 0 && nelems > 200) {
                        /* Protect from SET OF NULL compression bombs. */
                        ASN__DECODE_FAILED;
                    }
                    continue;
                }
                ASN_DEBUG("Failed to add element into %s",
                          td->name);
                /* Fall through */
                rv.code = RC_FAIL;
            } else {
                ASN_DEBUG("Failed decoding %s of %s (SET OF)",
                          elm->type->name, td->name);
            }
            if(ptr) ASN_STRUCT_FREE(*elm->type, ptr);
            return rv;
        }

        nelems = -1;  /* Allow uper_get_length() */
    } while(repeat);

    ASN_DEBUG("Decoded %s as SET OF", td->name);

    rv.code = RC_OK;
    rv.consumed = 0;
    return rv;
}

asn_enc_rval_t
SET_OF_encode_uper(const asn_TYPE_descriptor_t *td,
                   const asn_per_constraints_t *constraints, const void *sptr,
                   asn_per_outp_t *po) {
    const asn_anonymous_set_ *list;
    const asn_per_constraint_t *ct;
    const asn_TYPE_member_t *elm = td->elements;
    struct _el_buffer *encoded_els;
    asn_enc_rval_t er = {0,0,0};
    size_t encoded_edx;

    if(!sptr) ASN__ENCODE_FAILED;

    list = _A_CSET_FROM_VOID(sptr);

    er.encoded = 0;

    ASN_DEBUG("Encoding %s as SEQUENCE OF (%d)", td->name, list->count);

    if(constraints) ct = &constraints->size;
    else if(td->encoding_constraints.per_constraints)
        ct = &td->encoding_constraints.per_constraints->size;
    else ct = 0;

    /* If extensible constraint, check if size is in root */
    if(ct) {
        int not_in_root =
            (list->count < ct->lower_bound || list->count > ct->upper_bound);
        ASN_DEBUG("lb %"ASN_PRIdMAX" ub %"ASN_PRIdMAX" %s", ct->lower_bound, ct->upper_bound,
                  ct->flags & APC_EXTENSIBLE ? "ext" : "fix");
        if(ct->flags & APC_EXTENSIBLE) {
            /* Declare whether size is in extension root */
            if(per_put_few_bits(po, not_in_root, 1)) ASN__ENCODE_FAILED;
            if(not_in_root) ct = 0;
        } else if(not_in_root && ct->effective_bits >= 0) {
            ASN__ENCODE_FAILED;
        }

    }

    if(ct && ct->effective_bits >= 0) {
        /* X.691, #19.5: No length determinant */
        if(per_put_few_bits(po, list->count - ct->lower_bound,
                            ct->effective_bits))
            ASN__ENCODE_FAILED;
    } else if(list->count == 0) {
        /* When the list is empty add only the length determinant
         * X.691, #20.6 and #11.9.4.1
         */
        if (uper_put_length(po, 0, 0)) {
            ASN__ENCODE_FAILED;
        }
        ASN__ENCODED_OK(er);
    }


    /*
     * Canonical UPER #22.1 mandates dynamic sorting of the SET OF elements
     * according to their encodings. Build an array of the encoded elements.
     */
    encoded_els = SET_OF__encode_sorted(elm, list, SOES_CUPER);

    for(encoded_edx = 0; (ssize_t)encoded_edx < list->count;) {
        ssize_t may_encode;
        size_t edx;
        int need_eom = 0;

        if(ct && ct->effective_bits >= 0) {
            may_encode = list->count;
        } else {
            may_encode =
                uper_put_length(po, list->count - encoded_edx, &need_eom);
            if(may_encode < 0) ASN__ENCODE_FAILED;
        }

        for(edx = encoded_edx; edx < encoded_edx + may_encode; edx++) {
            const struct _el_buffer *el = &encoded_els[edx];
            if(asn_put_many_bits(po, el->buf,
                                 (8 * el->length) - el->bits_unused) < 0) {
                break;
            }
        }

        if(need_eom && uper_put_length(po, 0, 0))
            ASN__ENCODE_FAILED;  /* End of Message length */

        encoded_edx += may_encode;
    }

    SET_OF__encode_sorted_free(encoded_els, list->count);

    if((ssize_t)encoded_edx == list->count) {
        ASN__ENCODED_OK(er);
    } else {
        ASN__ENCODE_FAILED;
    }
}
