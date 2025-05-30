/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "VAM-PDU-Descriptions"
 * 	found in "/input/VAM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_vam_ts_VruClusterOperationContainer_H_
#define	_vam_ts_VruClusterOperationContainer_H_


#include <etsi_its_vam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_vam_ts_coding/vam_ts_DeltaTimeQuarterSecond.h"
#include <etsi_its_vam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct vam_ts_ClusterJoinInfo;
struct vam_ts_ClusterLeaveInfo;
struct vam_ts_ClusterBreakupInfo;

/* vam_ts_VruClusterOperationContainer */
typedef struct vam_ts_VruClusterOperationContainer {
	struct vam_ts_ClusterJoinInfo	*clusterJoinInfo;	/* OPTIONAL */
	struct vam_ts_ClusterLeaveInfo	*clusterLeaveInfo;	/* OPTIONAL */
	struct vam_ts_ClusterBreakupInfo	*clusterBreakupInfo;	/* OPTIONAL */
	vam_ts_DeltaTimeQuarterSecond_t	*clusterIdChangeTimeInfo;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} vam_ts_VruClusterOperationContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_vam_ts_VruClusterOperationContainer;
extern asn_SEQUENCE_specifics_t asn_SPC_vam_ts_VruClusterOperationContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_vam_ts_VruClusterOperationContainer_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_vam_ts_coding/vam_ts_ClusterJoinInfo.h"
#include "etsi_its_vam_ts_coding/vam_ts_ClusterLeaveInfo.h"
#include "etsi_its_vam_ts_coding/vam_ts_ClusterBreakupInfo.h"

#endif	/* _vam_ts_VruClusterOperationContainer_H_ */
#include <etsi_its_vam_ts_coding/asn_internal.h>
