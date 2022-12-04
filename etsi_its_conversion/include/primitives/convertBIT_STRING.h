#pragma once

#include <BIT_STRING.h>
#include <OCTET_STRING.h>
#include <string>

namespace etsi_its_cam_conversion
{
	void convert_BIT_STRINGtoRos(const BIT_STRING_t& _BIT_STRING_in, std::string& BIT_STRING_out)
	{
		for (int i = 0; i < _BIT_STRING_in.size; i++)
		{
			BIT_STRING_out.push_back(_BIT_STRING_in.buf[i]);
		}
	}

	void convert_BIT_STRINGtoC(const std::string& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out)
	{
		const char *cstr = _BIT_STRING_in.c_str();
		//int ret_v = OCTET_STRING_fromString(&BIT_STRING_out, cstr);
	}
}