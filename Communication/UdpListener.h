/*
* UdpListener.h

*
*  Created on: May 26, 2016
*      Author: laptopyellow
*/


#ifndef UDPLISTENER_H_
#define UDPLISTENER_H_
#include "../Brain/BrainThread.h"
#include "CommUtils.h"
//#include "message_enum.h"
#include "RoboCupGameControlData.h"
#include "UdpSender.h"
//#include "../Common/PlayerInfo.h"
#include <thread>
#include "UdpUtils.h"
//#include "../Common/SharedMemory.h"

struct MessageInfo
{
	uint8_t messageNumber; //
	uint16_t extraData; //like time or number
};

static MessageInfo new_info;
class UdpListener {

public:
	void Listen();
	static UdpListener* GetInstance();
	void inline stopThread();
	/*SharedMemory<RoboCupGameControlData>& GetData();
	SharedMemory<bool>& GetIsChangedFlag();
	void SetIsChangedFlag(const bool& flag);*/
	enum RefereeInfo
	{
		// type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
		Game_RoundRobin,
		Game_PlayOff,
		Game_dropin,
		// state of the game (STATE_READY, STATE_PLAYING, etc)      
		State_Initial,
		State_Ready,
		State_Set,
		State_Playing,
		State_Finished,

		First_Half,            // 1 = game in first half, 0 otherwise
		Second_Half,
		KickOffTeam,          // the team number of the next team to kick off or DROPBALL
		  // extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
		  State2_Normal,
		  State2_PenaltyShoot,
		  State2_Overtime,
		  State2_Timeout,
		  State2_Freekick,
		  State2_PenaltyKick,

		  //secondaryStateInfo[4],   // Extra info on the secondary state
		  DropInTeam,           // number of team that caused last drop in
		  DropInTime,          // number of seconds passed since the last drop in. -1 (0xffff) before first dropin
		  secsRemaining,       // estimate of number of seconds remaining in the half
		  secondaryTime,       // number of seconds shown as secondary time (remaining ready, until free ball, etc)

		   //penalty_info
		   Ball_Manipulation,
		   Physical_Contact,
		   Illegal_Attack,
		   Illegal_Defense,
		   Pickup_Or_Incapable,
		   Service,
	};


private:

	UdpListener();
	~UdpListener();
	void Init();
	int Receive();
	enum HeaderType { RGme = 0, unknown = -1 };
//	HeaderType GetHeader();
//	HeaderType ParseHeader();
	bool is_data_changed(RoboCupGameControlData* old_data, RoboCupGameControlData* new_data);
	//bool check_new_info(RoboCupGameControlData* old_data, RoboCupGameControlData* new_data);
	//void RaiseDataChanged();
	//inline void WriteGameData(const RoboCupGameControlData& data);

	enum { BUFFER_LENGTH = 1000, HEADER_LENGTH = 4 };
	const char* m_port_number;
	int m_socket_fd;
	char m_byte_buffer[BUFFER_LENGTH];
	char m_header[HEADER_LENGTH + 1];
	const int m_desired_version;

	sockaddr_storage m_their_addr;
	socklen_t m_addr_len;

	bool m_stop;
	std::thread m_listen_thread;
	static UdpListener* m_instance;

	/*SharedMemory<RoboCupGameControlData> mData;
	SharedMemory<bool> mIsDataChanged;*/

};



#endif /* UDPLISTENER_H_ */
