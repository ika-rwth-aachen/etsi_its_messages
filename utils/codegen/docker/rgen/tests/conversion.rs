mod utils;

e2e_hs!(
    single_byte,
    r#" SingleByte ::= INTEGER (0..255)"#,
    r#" 
#pragma once
#include <stdexcept>

#include <etsi_its_test_coding/SingleByte.h>
#include <etsi_its_test_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_test_msgs/SingleByte.hpp>
namespace test_msgs = etsi_its_test_msgs;
#else
#include <etsi_its_test_msgs/msg/single_byte.hpp>
namespace test_msgs = etsi_its_test_msgs::msg;
#endif 

namespace etsi_its_test_conversion {

void toRos_SingleByte(const SingleByte_t& in, test_msgs::SingleByte& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SingleByte(const test_msgs::SingleByte& in, SingleByte_t& out) {
  memset(&out, 0, sizeof(SingleByte_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}"#
);

e2e_hs!(
    integer_unconstrained,
    r#" Unbound ::= INTEGER "#,
    r#" 
#pragma once
#include <stdexcept>

#include <etsi_its_test_coding/Unbound.h>
#include <etsi_its_test_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_test_msgs/Unbound.hpp>
namespace test_msgs = etsi_its_test_msgs;
#else
#include <etsi_its_test_msgs/msg/unbound.hpp>
namespace test_msgs = etsi_its_test_msgs::msg;
#endif 

namespace etsi_its_test_conversion {

void toRos_Unbound(const Unbound_t& in, test_msgs::Unbound& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_Unbound(const test_msgs::Unbound& in, Unbound_t& out) {
  memset(&out, 0, sizeof(Unbound_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}"#
);

e2e_hs!(
    sequence,
    r#" Seq ::= SEQUENCE { aBigNumber INTEGER, anotherBigNumber INTEGER} "#,
    r#" 
#pragma once
#include <stdexcept>

#include <etsi_its_test_coding/Seq.h>
#include <etsi_its_test_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#include <etsi_its_test_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_test_msgs/Seq.hpp>
namespace test_msgs = etsi_its_test_msgs;
#else
#include <etsi_its_test_msgs/msg/seq.hpp>
namespace test_msgs = etsi_its_test_msgs::msg;
#endif 

namespace etsi_its_test_conversion {

void toRos_Seq(const Seq_t& in, test_msgs::Seq& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in.aBigNumber, out.a_big_number);
  etsi_its_primitives_conversion::toRos_INTEGER(in.anotherBigNumber, out.another_big_number);
}

void toStruct_Seq(const test_msgs::Seq& in, Seq_t& out) {
  memset(&out, 0, sizeof(Seq_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.a_big_number, out.aBigNumber);
  etsi_its_primitives_conversion::toStruct_INTEGER(in.another_big_number, out.anotherBigNumber);
}

}"#
);

e2e_hs!(boolean, 
    r#" Maybe ::= BOOLEAN "#, 
    r#" 
#pragma once
#include <stdexcept>

#include <etsi_its_test_coding/Maybe.h>
#include <etsi_its_test_coding/BOOLEAN.h>
#include <etsi_its_primitives_conversion/convertBOOLEAN.h>
#ifdef ROS1
#include <etsi_its_test_msgs/Maybe.hpp>
namespace test_msgs = etsi_its_test_msgs;
#else
#include <etsi_its_test_msgs/msg/maybe.hpp>
namespace test_msgs = etsi_its_test_msgs::msg;
#endif 

namespace etsi_its_test_conversion {

void toRos_Maybe(const Maybe_t& in, test_msgs::Maybe& out) {
  etsi_its_primitives_conversion::toRos_BOOLEAN(in, out.value);
}

void toStruct_Maybe(const test_msgs::Maybe& in, Maybe_t& out) {
  memset(&out, 0, sizeof(Maybe_t));

  etsi_its_primitives_conversion::toStruct_BOOLEAN(in.value, out);
}

}"#
);
