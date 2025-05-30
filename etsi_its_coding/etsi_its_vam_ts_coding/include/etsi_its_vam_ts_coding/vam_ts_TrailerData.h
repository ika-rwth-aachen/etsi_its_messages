/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_vam_ts_TrailerData_H_
#define	_vam_ts_TrailerData_H_


#include <etsi_its_vam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_vam_ts_coding/vam_ts_Identifier1B.h"
#include "etsi_its_vam_ts_coding/vam_ts_StandardLength1B.h"
#include "etsi_its_vam_ts_coding/vam_ts_VehicleWidth.h"
#include "etsi_its_vam_ts_coding/vam_ts_CartesianAngle.h"
#include <etsi_its_vam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* vam_ts_TrailerData */
typedef struct vam_ts_TrailerData {
	vam_ts_Identifier1B_t	 refPointId;
	vam_ts_StandardLength1B_t	 hitchPointOffset;
	vam_ts_StandardLength1B_t	*frontOverhang;	/* OPTIONAL */
	vam_ts_StandardLength1B_t	*rearOverhang;	/* OPTIONAL */
	vam_ts_VehicleWidth_t	*trailerWidth;	/* OPTIONAL */
	vam_ts_CartesianAngle_t	 hitchAngle;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} vam_ts_TrailerData_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_vam_ts_TrailerData;

#ifdef __cplusplus
}
#endif

#endif	/* _vam_ts_TrailerData_H_ */
#include <etsi_its_vam_ts_coding/asn_internal.h>
