#pragma once

#include <INTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_toRos(const INTEGER_t& _INTEGER_in, long& INTEGER_out)
	{
		int ret_v = asn_INTEGER2long(&_INTEGER_in, &INTEGER_out);
	}

	void convert_toRos(const long& _INTEGER_in, long& INTEGER_out)
	{
        INTEGER_out = _INTEGER_in;
	}

	void convert_toC(const long& _INTEGER_in, INTEGER_t& INTEGER_out)
	{
		int ret_v = asn_long2INTEGER(&INTEGER_out, _INTEGER_in);
	}

	void convert_toC(const long& _INTEGER_in, long& INTEGER_out)
	{
        INTEGER_out = _INTEGER_in;
	}

}