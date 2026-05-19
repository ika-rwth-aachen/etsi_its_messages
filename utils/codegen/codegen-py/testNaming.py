#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

import unittest

from asn1CodeGenerationUtils import validCFieldAsGenByAsn1c, validRosField, validRosType


class TestNaming(unittest.TestCase):

    def test_validCField(self):
        self.assertEqual(validCFieldAsGenByAsn1c("a"), "a")
        self.assertEqual(validCFieldAsGenByAsn1c("field-name"), "field_name")
        self.assertEqual(validCFieldAsGenByAsn1c("class"), "Class")
        self.assertEqual(validCFieldAsGenByAsn1c("long"), "Long")

    def test_validRosType(self):
        self.assertEqual(validRosType("a"), "A")
        self.assertEqual(validRosType("abc"), "Abc")
        self.assertEqual(validRosType("typeName"), "TypeName")
        self.assertEqual(validRosType("TypeName"), "TypeName")
        self.assertEqual(validRosType("Type_Name"), "TypeName")
        self.assertEqual(validRosType("TYPE_NAME"), "TYPENAME")
        self.assertEqual(validRosType("Type_NAME_123"), "TypeNAME123")
        self.assertEqual(validRosType("1TypeName"), "A1TypeName")
        self.assertEqual(validRosType("uint32[]"), "uint32[]")

    def test_validRosField(self):
        self.assertEqual(validRosField("a"), "a")
        self.assertEqual(validRosField("B"), "b")
        self.assertEqual(validRosField("abc"), "abc")
        self.assertEqual(validRosField("ABC"), "abc")
        self.assertEqual(validRosField("a_b_c_"), "a_b_c_")
        self.assertEqual(validRosField("a-b-c"), "a_b_c")
        self.assertEqual(validRosField("fieldName"), "field_name")
        self.assertEqual(validRosField("FieldName"), "field_name")
        self.assertEqual(validRosField("FIELD_NAME"), "field_name")
        self.assertEqual(validRosField("field-name"), "field_name")
        self.assertEqual(validRosField("FieldName123"), "field_name123")
        self.assertEqual(validRosField("FieldABCName"), "field_abc_name")
        self.assertEqual(validRosField("FieldABC123Name"), "field_abc123_name")
        self.assertEqual(validRosField("class"), "cls")
        self.assertEqual(validRosField("long"), "lon")

    def test_validRosFieldConst(self):
        self.assertEqual(validRosField("a", is_const=True), "A")
        self.assertEqual(validRosField("B", is_const=True), "B")
        self.assertEqual(validRosField("abc", is_const=True), "ABC")
        self.assertEqual(validRosField("ABC", is_const=True), "ABC")
        self.assertEqual(validRosField("a_b_c_", is_const=True), "A_B_C_")
        self.assertEqual(validRosField("a-b-c", is_const=True), "A_B_C")
        self.assertEqual(validRosField("fieldName", is_const=True), "FIELD_NAME")
        self.assertEqual(validRosField("FieldName", is_const=True), "FIELD_NAME")
        self.assertEqual(validRosField("FIELD_NAME", is_const=True), "FIELD_NAME")
        self.assertEqual(validRosField("field-name", is_const=True), "FIELD_NAME")
        self.assertEqual(validRosField("FieldName123", is_const=True), "FIELD_NAME123")
        self.assertEqual(validRosField("FieldABCName", is_const=True), "FIELD_ABC_NAME")
        self.assertEqual(validRosField("FieldABC123Name", is_const=True), "FIELD_ABC123_NAME")


if __name__ == "__main__":
    unittest.main()
