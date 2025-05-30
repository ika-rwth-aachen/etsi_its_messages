/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ActionID_H_
#define	_cam_ActionID_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_cam_coding/cam_StationID.h"
#include "etsi_its_cam_coding/cam_SequenceNumber.h"
#include <etsi_its_cam_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* cam_ActionID */
typedef struct cam_ActionID {
	cam_StationID_t	 originatingStationID;
	cam_SequenceNumber_t	 sequenceNumber;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ActionID_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ActionID;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ActionID_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
