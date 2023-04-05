#pragma once

#include <etsi_its_cam_coding/BIT_STRING.h>
#include <etsi_its_cam_coding/OCTET_STRING.h>
#include <string>

namespace etsi_its_cam_conversion
{
	void convert_BIT_STRINGtoRos(const BIT_STRING_t& _BIT_STRING_in, std::vector<uint8_t>& BIT_STRING_out)
	{
		BIT_STRING_out.resize(_BIT_STRING_in.size);
		for (int i = 0; i < _BIT_STRING_in.size; i++)
		{
			BIT_STRING_out[i] = _BIT_STRING_in.buf[i];
		}
	}

	void convert_BIT_STRINGtoC(const std::vector<uint8_t>& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out)
	{
		BIT_STRING_out.bits_unused = 0;
		BIT_STRING_out.size = _BIT_STRING_in.size();
		BIT_STRING_out.buf = new uint8_t[BIT_STRING_out.size];
		memset(BIT_STRING_out.buf, 0, BIT_STRING_out.size); // Initialize buffer to 0

		for (size_t i = 0; i < _BIT_STRING_in.size(); i++) {
			BIT_STRING_out.buf[i] = _BIT_STRING_in[i];
    	}
	}
}