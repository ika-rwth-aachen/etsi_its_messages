/*
 * Copyright (c) 2017 Lev Walkin <vlm@lionet.info>.
 * All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
#include <etsi_its_mcm_uulm_coding/NULL.h>

asn_dec_rval_t
NULL_decode_uper(const asn_codec_ctx_t *opt_codec_ctx,
                 const asn_TYPE_descriptor_t *td,
                 const asn_per_constraints_t *constraints, void **sptr,
                 asn_per_data_t *pd) {
    asn_dec_rval_t rv;

    (void)opt_codec_ctx;
    (void)td;
    (void)constraints;
    (void)pd;

    if(!*sptr) {
        *sptr = MALLOC(sizeof(NULL_t));
        if(*sptr) {
            *(NULL_t *)*sptr = 0;
        } else {
            ASN__DECODE_FAILED;
        }
    }

    /*
     * NULL type does not have content octets.
     */

    rv.code = RC_OK;
    rv.consumed = 0;
    return rv;
}

asn_enc_rval_t
NULL_encode_uper(const asn_TYPE_descriptor_t *td,
                 const asn_per_constraints_t *constraints, const void *sptr,
                 asn_per_outp_t *po) {
    asn_enc_rval_t er = {0,0,0};

    (void)td;
    (void)constraints;
    (void)sptr;
    (void)po;

    er.encoded = 0;
    ASN__ENCODED_OK(er);
}
