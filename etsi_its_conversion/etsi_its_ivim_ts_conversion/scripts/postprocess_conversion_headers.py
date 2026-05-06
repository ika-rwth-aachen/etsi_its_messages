#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path


TYPE_ALIASES = {
    "DDDIO": "DDD_IO",
    "DDDIOLIST": "DDD_IO_LIST",
    "InternationalSignapplicablePeriod": "InternationalSign_applicablePeriod",
    "InternationalSigndistanceBetweenVehicles": "InternationalSign_distanceBetweenVehicles",
    "InternationalSignexemptedApplicablePeriod": "InternationalSign_exemptedApplicablePeriod",
    "InternationalSigndestinationInformation": "InternationalSign_destinationInformation",
    "DriverCharacteristics": "IVI_DriverCharacteristics",
    "TrailerCharacteristics": "IVI_TrailerCharacteristics",
    "StationType": "ITS_Container_StationType",
    "Temperature2": "IVI_Temperature",
}

HEADER_ALIASES = {
    "BIT STRING": "BIT_STRING",
    "OCTET STRING": "OCTET_STRING",
    "DDDIO": "DDD-IO",
    "DDDIOLIST": "DDD-IO-LIST",
    "InternationalSignapplicablePeriod": "InternationalSign-applicablePeriod",
    "InternationalSigndistanceBetweenVehicles": "InternationalSign-distanceBetweenVehicles",
    "InternationalSignexemptedApplicablePeriod": "InternationalSign-exemptedApplicablePeriod",
    "InternationalSigndestinationInformation": "InternationalSign-destinationInformation",
    "DriverCharacteristics": "IVI_DriverCharacteristics",
    "TrailerCharacteristics": "IVI_TrailerCharacteristics",
    "StationType": "ITS-Container_StationType",
    "Temperature2": "IVI_Temperature",
}


def default_repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def rewrite_header(name: str, text: str) -> str:
    for src, dst in HEADER_ALIASES.items():
        text = text.replace(f"ivim_ts_{src}.h", f"ivim_ts_{dst}.h")
    for src, dst in TYPE_ALIASES.items():
        text = text.replace(f"ivim_ts_{src}_t", f"ivim_ts_{dst}_t")
        text = text.replace(f"sizeof(ivim_ts_{src}_t)", f"sizeof(ivim_ts_{dst}_t)")

    text = text.replace("ivim_ts_IVI-TrailerCharacteristics.h", "ivim_ts_IVI_TrailerCharacteristics.h")
    text = text.replace("ivim_ts_IVI-DriverCharacteristics.h", "ivim_ts_IVI_DriverCharacteristics.h")
    text = text.replace("ivim_ts_IVI-Temperature.h", "ivim_ts_IVI_Temperature.h")
    text = text.replace("ivim_ts_InternationalSign_destinationInformation.h", "ivim_ts_InternationalSign-destinationInformation.h")
    text = text.replace("ivim_ts_InternationalSign_distanceBetweenVehicles.h", "ivim_ts_InternationalSign-distanceBetweenVehicles.h")
    text = text.replace("ivim_ts_InternationalSign_exemptedApplicablePeriod.h", "ivim_ts_InternationalSign-exemptedApplicablePeriod.h")

    text = text.replace("toRos_BIT STRING", "toRos_BIT_STRING")
    text = text.replace("toStruct_BIT STRING", "toStruct_BIT_STRING")
    text = text.replace("toRos_OCTET STRING", "toRos_OCTET_STRING")
    text = text.replace("toStruct_OCTET STRING", "toStruct_OCTET_STRING")
    text = text.replace("convertBIT STRING.h", "convertBIT_STRING.h")
    text = text.replace("convertOCTET STRING.h", "convertOCTET_STRING.h")
    text = re.sub(r'#include <etsi_its_ivim_ts_conversion/convertuint\d+\.h>\n', "", text)
    text = re.sub(r'\btoRos_uint\d+\(([^,]+),\s*([^)]+)\);', r'\2 = \1;', text)
    text = re.sub(r'\btoStruct_uint\d+\(([^,]+),\s*([^)]+)\);', r'\2 = \1;', text)

    if name == "convertText.h":
        text = re.sub(
            r'  for \(int i = 0; i < in\.language\.list\.count; \+\+i\) \{\n'
            r'    ivim_ts_msgs::uint8 el;\n'
            r'    el = \*\(in\.language\.list\.array\[i\]\);\n'
            r'    out\.language\.push_back\(el\);\n'
            r'  \}\n',
            "  etsi_its_primitives_conversion::toRos_BIT_STRING(in.language, out.language);\n"
            "  out.bits_unused = in.language.bits_unused;\n",
            text,
        )
        text = re.sub(
            r'    for \(int i = 0; i < in\.language\.size\(\); \+\+i\) \{\n'
            r'      ivim_ts_uint8_t\* el = \(ivim_ts_uint8_t\*\) calloc\(1, sizeof\(ivim_ts_uint8_t\)\);\n'
            r'      \*el = in\.language\[i\];\n'
            r'      if \(asn_sequence_add\(&out\.language, el\)\) throw std::invalid_argument\("Failed to add to A_SEQUENCE_OF"\);\n'
            r'    \}\n',
            "  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.language, out.language);\n"
            "  out.language.bits_unused = in.bits_unused;\n",
            text,
        )

    if name == "convertTcPart.h":
        text = re.sub(
            r'  for \(int i = 0; i < in\.data\.list\.count; \+\+i\) \{\n'
            r'    ivim_ts_msgs::uint8 el;\n'
            r'    el = \*\(in\.data\.list\.array\[i\]\);\n'
            r'    out\.data\.push_back\(el\);\n'
            r'  \}\n'
            r'  toRos_IviType\(in\.iviType, out\.ivi_type\);\n'
            r'  if \(in\.laneStatus\) \{\n'
            r'    toRos_LaneStatus\(\*in\.laneStatus, out\.lane_status\);\n'
            r'    out\.lane_status_is_present = true;\n'
            r'  \}\n'
            r'  if \(in\.vehicleCharacteristics\) \{\n'
            r'    toRos_VehicleCharacteristicsList\(\*in\.vehicleCharacteristics, out\.vehicle_characteristics\);\n'
            r'    out\.vehicle_characteristics_is_present = true;\n'
            r'  \}\n',
            "  etsi_its_primitives_conversion::toRos_OCTET_STRING(in.data, out.data);\n"
            "  if (in.ext1) {\n"
            "    toRos_IviType(in.ext1->iviType, out.ivi_type);\n"
            "    if (in.ext1->laneStatus) {\n"
            "      toRos_LaneStatus(*in.ext1->laneStatus, out.lane_status);\n"
            "      out.lane_status_is_present = true;\n"
            "    }\n"
            "    if (in.ext1->vehicleCharacteristics) {\n"
            "      toRos_VehicleCharacteristicsList(*in.ext1->vehicleCharacteristics, out.vehicle_characteristics);\n"
            "      out.vehicle_characteristics_is_present = true;\n"
            "    }\n"
            "  }\n",
            text,
        )
        text = re.sub(
            r'    for \(int i = 0; i < in\.data\.size\(\); \+\+i\) \{\n'
            r'      ivim_ts_uint8_t\* el = \(ivim_ts_uint8_t\*\) calloc\(1, sizeof\(ivim_ts_uint8_t\)\);\n'
            r'      \*el = in\.data\[i\];\n'
            r'      if \(asn_sequence_add\(&out\.data, el\)\) throw std::invalid_argument\("Failed to add to A_SEQUENCE_OF"\);\n'
            r'    \}\n'
            r'  toStruct_IviType\(in\.ivi_type, out\.iviType\);\n'
            r'  if \(in\.lane_status_is_present\) \{\n'
            r'    out\.laneStatus = \(ivim_ts_LaneStatus_t\*\) calloc\(1, sizeof\(ivim_ts_LaneStatus_t\)\);\n'
            r'    toStruct_LaneStatus\(in\.lane_status, \*out\.laneStatus\);\n'
            r'  \}\n'
            r'  if \(in\.vehicle_characteristics_is_present\) \{\n'
            r'    out\.vehicleCharacteristics = \(ivim_ts_VehicleCharacteristicsList_t\*\) calloc\(1, sizeof\(ivim_ts_VehicleCharacteristicsList_t\)\);\n'
            r'    toStruct_VehicleCharacteristicsList\(in\.vehicle_characteristics, \*out\.vehicleCharacteristics\);\n'
            r'  \}\n',
            "  etsi_its_primitives_conversion::toStruct_OCTET_STRING(in.data, out.data);\n"
            "  if (!out.ext1) out.ext1 = static_cast<decltype(out.ext1)>(calloc(1, sizeof(*out.ext1)));\n"
            "  toStruct_IviType(in.ivi_type, out.ext1->iviType);\n"
            "  if (in.lane_status_is_present) {\n"
            "    out.ext1->laneStatus = (ivim_ts_LaneStatus_t*) calloc(1, sizeof(ivim_ts_LaneStatus_t));\n"
            "    toStruct_LaneStatus(in.lane_status, *out.ext1->laneStatus);\n"
            "  }\n"
            "  if (in.vehicle_characteristics_is_present) {\n"
            "    out.ext1->vehicleCharacteristics = (ivim_ts_VehicleCharacteristicsList_t*) calloc(1, sizeof(ivim_ts_VehicleCharacteristicsList_t));\n"
            "    toStruct_VehicleCharacteristicsList(in.vehicle_characteristics, *out.ext1->vehicleCharacteristics);\n"
            "  }\n",
            text,
        )
        text = text.replace(
            "  toStruct_IviType(in.ivi_type, out.iviType);\n"
            "  if (in.lane_status_is_present) {\n"
            "    out.laneStatus = (ivim_ts_LaneStatus_t*) calloc(1, sizeof(ivim_ts_LaneStatus_t));\n"
            "    toStruct_LaneStatus(in.lane_status, *out.laneStatus);\n"
            "  }\n"
            "  if (in.vehicle_characteristics_is_present) {\n"
            "    out.vehicleCharacteristics = (ivim_ts_VehicleCharacteristicsList_t*) calloc(1, sizeof(ivim_ts_VehicleCharacteristicsList_t));\n"
            "    toStruct_VehicleCharacteristicsList(in.vehicle_characteristics, *out.vehicleCharacteristics);\n"
            "  }\n",
            "  if (!out.ext1) out.ext1 = static_cast<decltype(out.ext1)>(calloc(1, sizeof(*out.ext1)));\n"
            "  toStruct_IviType(in.ivi_type, out.ext1->iviType);\n"
            "  if (in.lane_status_is_present) {\n"
            "    out.ext1->laneStatus = (ivim_ts_LaneStatus_t*) calloc(1, sizeof(ivim_ts_LaneStatus_t));\n"
            "    toStruct_LaneStatus(in.lane_status, *out.ext1->laneStatus);\n"
            "  }\n"
            "  if (in.vehicle_characteristics_is_present) {\n"
            "    out.ext1->vehicleCharacteristics = (ivim_ts_VehicleCharacteristicsList_t*) calloc(1, sizeof(ivim_ts_VehicleCharacteristicsList_t));\n"
            "    toStruct_VehicleCharacteristicsList(in.vehicle_characteristics, *out.ext1->vehicleCharacteristics);\n"
            "  }\n",
        )

    if name == "convertLaneInformation.h":
        for field in [
            ("detectionZoneIds", "detection_zone_ids", "ZoneIds"),
            ("relevanceZoneIds", "relevance_zone_ids", "ZoneIds"),
            ("laneCharacteristics", "lane_characteristics", "LaneCharacteristics"),
            ("laneSurfaceStaticCharacteristics", "lane_surface_static_characteristics", "RoadSurfaceStaticCharacteristics"),
            ("laneSurfaceDynamicCharacteristics", "lane_surface_dynamic_characteristics", "RoadSurfaceDynamicCharacteristics"),
        ]:
            c_name, ros_name, conv = field
            text = text.replace(f"if (in.{c_name}) {{", f"if (in.ext1 && in.ext1->{c_name}) {{")
            text = text.replace(f"*in.{c_name}", f"*in.ext1->{c_name}")
            text = text.replace(f"out.{c_name} = (ivim_ts_{conv}_t*) calloc(1, sizeof(ivim_ts_{conv}_t));", f"if (!out.ext1) out.ext1 = static_cast<decltype(out.ext1)>(calloc(1, sizeof(*out.ext1)));\n    out.ext1->{c_name} = (ivim_ts_{conv}_t*) calloc(1, sizeof(ivim_ts_{conv}_t));")
            text = text.replace(f"*out.{c_name}", f"*out.ext1->{c_name}")

    if name == "convertInternationalSignapplicablePeriod.h":
        for tag in ("year", "month_day", "hourMinutes"):
            text = text.replace(
                f"(struct ivim_ts_InternationalSign_applicablePeriod__{tag}*) calloc(1, sizeof(*out.{tag}))",
                f"static_cast<decltype(out.{tag})>(calloc(1, sizeof(*out.{tag})))",
            )

    if name in ("convertInternationalSigndistanceBetweenVehicles.h", "convertInternationalSignexemptedApplicablePeriod.h"):
        text = text.replace("ivim_ts_InternationalSign-", "ivim_ts_InternationalSign_")

    if name == "convertVehicleCharacteristicsRanges.h":
        text = text.replace("in.limits.numberOfAxles", "in.limits.choice.numberOfAxles")
        text = text.replace("out.limits.numberOfAxles", "out.limits.choice.numberOfAxles")
        text = text.replace("in.limits.limits.present", "in.limits.present")
        text = text.replace("in.limits.limits.choice.", "in.limits.choice.")
        text = text.replace("out.limits.limits.present", "out.limits.present")
        text = text.replace("out.limits.limits.choice.", "out.limits.choice.")
        text = text.replace("ivim_ts_VehicleCharacteristicsRanges__limits__limits_PR", "ivim_ts_VehicleCharacteristicsRanges__limits_PR")

    if name == "convertTrailerCharacteristicsList.h":
        text = text.replace(
            "toRos_TrailerCharacteristics(*(in.list.array[i]), el);",
            "toRos_TrailerCharacteristics(*reinterpret_cast<const ivim_ts_IVI_TrailerCharacteristics_t*>(in.list.array[i]), el);",
        )
        text = text.replace(
            "ivim_ts_IVI_TrailerCharacteristics_t* el = (ivim_ts_IVI_TrailerCharacteristics_t*) calloc(1, sizeof(ivim_ts_IVI_TrailerCharacteristics_t));",
            "auto* el = (struct ivim_ts_TrailerCharacteristics*) calloc(1, sizeof(ivim_ts_IVI_TrailerCharacteristics_t));",
        )
        text = text.replace(
            "toStruct_TrailerCharacteristics(in.array[i], *el);",
            "toStruct_TrailerCharacteristics(in.array[i], *reinterpret_cast<ivim_ts_IVI_TrailerCharacteristics_t*>(el));",
        )

    if name == "convertCompleteVehicleCharacteristics.h":
        text = text.replace(
            "toRos_TrainCharacteristics(*in.train, out.train);",
            "toRos_TrainCharacteristics(reinterpret_cast<const ivim_ts_TrainCharacteristics_t&>(*in.train), out.train);",
        )
        text = text.replace(
            "out.train = (ivim_ts_TrainCharacteristics_t*) calloc(1, sizeof(ivim_ts_TrainCharacteristics_t));",
            "out.train = (struct ivim_ts_TrainCharacteristics*) calloc(1, sizeof(ivim_ts_TrainCharacteristics_t));",
        )
        text = text.replace(
            "toStruct_TrainCharacteristics(in.train, *out.train);",
            "toStruct_TrainCharacteristics(in.train, reinterpret_cast<ivim_ts_TrainCharacteristics_t&>(*out.train));",
        )

    if name == "convertGddAttribute.h":
        text = re.sub(
            r'    break;case ivim_ts_GddAttribute_PR_ddd:\n'
            r'    toRos_InternationalSigndestinationInformation\(in\.choice\.ddd, out\.ddd\);\n'
            r'    out\.choice = ivim_ts_msgs::GddAttribute::CHOICE_DDD;\n',
            "",
            text,
        )
        text = re.sub(
            r'    break;case ivim_ts_msgs::GddAttribute::CHOICE_DDD:\n'
            r'    toStruct_InternationalSigndestinationInformation\(in\.ddd, out\.choice\.ddd\);\n'
            r'    out\.present = ivim_ts_GddAttribute_PR_ddd;\n',
            "",
            text,
        )

    if name == "convertDDDIOLIST.h":
        text = text.replace("ivim_ts_DDD-IO-LIST_t", "ivim_ts_DDD_IO_LIST_t")
        text = text.replace("sizeof(ivim_ts_DDD-IO-LIST_t)", "sizeof(ivim_ts_DDD_IO_LIST_t)")

    if name == "convertRSCode.h":
        text = text.replace("ivim_ts_RSCode__code__code_PR_", "ivim_ts_RSCode__code_PR_")
        text = text.replace("in.code.itisCodes", "in.code.choice.itisCodes")
        text = text.replace("out.code.itisCodes", "out.code.choice.itisCodes")

    if name == "convertInternationalSigndestinationInformation.h":
        text = text.replace(
            "  toRos_DDDIOLIST(in.ioList, out.io_list);\n",
            "  toRos_DDDIOLIST(*in.ioList, out.io_list);\n",
        )
        text = text.replace(
            "  toStruct_DDDIOLIST(in.io_list, out.ioList);\n",
            "  out.ioList = (ivim_ts_DDD_IO_LIST_t*) calloc(1, sizeof(ivim_ts_DDD_IO_LIST_t));\n"
            "  toStruct_DDDIOLIST(in.io_list, *out.ioList);\n",
        )

    text = text.replace("ivim_ts_InternationalSign_destinationInformation.h", "ivim_ts_InternationalSign-destinationInformation.h")
    text = text.replace("ivim_ts_InternationalSign_distanceBetweenVehicles.h", "ivim_ts_InternationalSign-distanceBetweenVehicles.h")
    text = text.replace("ivim_ts_InternationalSign_exemptedApplicablePeriod.h", "ivim_ts_InternationalSign-exemptedApplicablePeriod.h")

    return text


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=default_repo_root())
    parser.add_argument("--report", type=Path, default=None)
    args = parser.parse_args()

    conv_dir = args.repo_root / "etsi_its_conversion/etsi_its_ivim_ts_conversion/include/etsi_its_ivim_ts_conversion"
    changed = 0

    for header in conv_dir.glob("convert*.h"):
        original = header.read_text(encoding="utf-8")
        text = rewrite_header(header.name, original)
        if text != original:
            header.write_text(text, encoding="utf-8")
            changed += 1

    report = f"IVIM conversion postprocess\nChanged files: {changed}\n"
    print(report)
    if args.report:
        args.report.write_text(report, encoding="utf-8")


if __name__ == "__main__":
    main()
