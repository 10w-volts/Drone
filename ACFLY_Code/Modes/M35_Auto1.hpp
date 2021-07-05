#pragma once

#include "Modes.hpp"

void mavlink_send_command_ack(ModeMsg msg,
															uint8_t result,
															uint8_t progress,
															int32_t result_param2);

class M35_Auto1:public Mode_Base 
{
	private:
		
	public:
		M35_Auto1();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};