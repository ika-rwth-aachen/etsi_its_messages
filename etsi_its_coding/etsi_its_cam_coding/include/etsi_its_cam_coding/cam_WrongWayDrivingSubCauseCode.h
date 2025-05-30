/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_WrongWayDrivingSubCauseCode_H_
#define	_cam_WrongWayDrivingSubCauseCode_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_WrongWayDrivingSubCauseCode {
	cam_WrongWayDrivingSubCauseCode_unavailable	= 0,
	cam_WrongWayDrivingSubCauseCode_wrongLane	= 1,
	cam_WrongWayDrivingSubCauseCode_wrongDirection	= 2
} e_cam_WrongWayDrivingSubCauseCode;

/* cam_WrongWayDrivingSubCauseCode */
typedef long	 cam_WrongWayDrivingSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_WrongWayDrivingSubCauseCode;
asn_struct_free_f cam_WrongWayDrivingSubCauseCode_free;
asn_struct_print_f cam_WrongWayDrivingSubCauseCode_print;
asn_constr_check_f cam_WrongWayDrivingSubCauseCode_constraint;
jer_type_encoder_f cam_WrongWayDrivingSubCauseCode_encode_jer;
per_type_decoder_f cam_WrongWayDrivingSubCauseCode_decode_uper;
per_type_encoder_f cam_WrongWayDrivingSubCauseCode_encode_uper;
per_type_decoder_f cam_WrongWayDrivingSubCauseCode_decode_aper;
per_type_encoder_f cam_WrongWayDrivingSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_WrongWayDrivingSubCauseCode_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
