/*
* UdpListener.cpp
*
*  Created on: May 26, 2016
*      Author: laptopyellow
*/

#include "UdpListener.h"
#include <signal.h>

#define FIRST_HALF          1
#define SECOND_HALF         0
#define NO_EXTRA_DATA       0
#define PLAYER_NUMBER       1
#define TEAM_NUMBER			17

UdpListener* UdpListener::m_instance = NULL;
MessageInfo UdpListener::new_info;

UdpListener* UdpListener::GetInstance() {
	if (m_instance == NULL)
	{
		m_instance = new UdpListener();
		return m_instance;
	}
	else
	{
		return m_instance;
	}
}

void inline UdpListener::stopThread()
{
	m_stop = true;
	//m_listen_thread.join();
}

UdpListener::UdpListener() : m_port_number("3838"),
m_desired_version(11),
m_addr_len(sizeof(m_their_addr)),
m_stop(false),
m_listen_thread(std::thread(&UdpListener::Listen, this))
{
	cout<<"UdpListener()"<<endl;
	m_header[HEADER_LENGTH] = '\0';

}

UdpListener::~UdpListener() {
	if (m_instance != NULL) {
		delete m_instance;
		close(m_socket_fd);
	}
}

void UdpListener::Init() {

	m_socket_fd = UdpUtils::CreateUdpReceiveSocket(NULL, m_port_number);
}

int UdpListener::Receive() {
	int numbytes;
	if ((numbytes = recvfrom(m_socket_fd, (void*)m_byte_buffer, BUFFER_LENGTH - 1, 0,
		(sockaddr *)&m_their_addr, &m_addr_len)) == -1) {
		throw CommunicationException("recvfrom");
	}
	return numbytes;
}

void UdpListener::Listen() {
	Init();
	cout << "start listening" << endl;
	RoboCupGameControlData oldData, newData;
	while (!m_stop) {
		Receive();
		HeaderType type;
		strncpy(m_header, m_byte_buffer, HEADER_LENGTH);
		if (strcmp(m_header, "RGme") == 0)
			type = RGme;
		else
			type = unknown;
		switch (type) {
		case RGme: {
			uint16_t version;
			memcpy(&version, m_byte_buffer + 4, sizeof(version));
			if (version == m_desired_version) {
				oldData = newData;
				memcpy(&newData, m_byte_buffer, sizeof(RoboCupGameControlData));
				if (is_data_changed(&oldData, &newData)) {
					cout << "Data changed!!!! " << endl;
					pthread_kill(BrainThread::GetBrainThreadInstance()->getBrainThread(), BrainThread::NEW_REFEREE_MESSAGE); //send signal to the brain thread - which will trigger the GetBallCenterInFrameAndDistance() method.
				}
				RoboCupGameControlReturnData returnData;
				returnData.player = PLAYER_NUMBER;
				returnData.team = TEAM_NUMBER;
				returnData.message = 0;
				char serilized_data[sizeof(RoboCupGameControlReturnData)];
				memcpy(serilized_data, &returnData, sizeof(returnData));
				UdpSender::GetInstance()->Send(serilized_data);
			}
			break;
		}
		case unknown: {cout << "unknown1" << endl; cout.flush(); break; }
		}
	}
}
bool UdpListener::is_data_changed(RoboCupGameControlData * old_data, RoboCupGameControlData * new_data)
{
	//checking whether interesting fields have been changed

	if (old_data->gameType != new_data->gameType) {
		switch (new_data->gameType)
		{
		case GAME_ROUNDROBIN:
			new_info.messageNumber = RefereeInfo::Game_RoundRobin;
			new_info.extraData = NO_EXTRA_DATA;
			cout<<"Game_RoundRobin"<<endl;
			return true;
		case GAME_PLAYOFF:
			cout<<"GAME_PLAYOFF"<<endl;
			new_info.messageNumber = RefereeInfo::Game_PlayOff;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case GAME_DROPIN:
			cout<<"GAME_DROPIN"<<endl;
			new_info.messageNumber = RefereeInfo::Game_dropin;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		default:
			break;
		}
	}
	if (old_data->state != new_data->state) {
		switch (new_data->state)
		{
		case STATE_INITIAL:
			cout<<"STATE_INITIAL"<<endl;
			new_info.messageNumber = RefereeInfo::State_Initial;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE_READY:
			cout<<"STATE_READY"<<endl;
			new_info.messageNumber = RefereeInfo::State_Ready;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE_SET:
			cout<<"STATE_SET"<<endl;
			new_info.messageNumber = RefereeInfo::State_Set;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE_PLAYING:
			cout<<"STATE_PLAYING"<<endl;
			new_info.messageNumber = RefereeInfo::State_Playing;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE_FINISHED:
			cout<<"STATE_FINISHED"<<endl;
			new_info.messageNumber = RefereeInfo::State_Finished;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		default:
			break;
		}
	}
	if (old_data->firstHalf != new_data->firstHalf) {
		switch (new_data->firstHalf)
		{
		case SECOND_HALF: //1 = game in first half, 0 otherwise
			new_info.messageNumber = RefereeInfo::Second_Half;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case FIRST_HALF:
			cout<<"FIRST_HALF"<<endl;
			new_info.messageNumber = RefereeInfo::First_Half;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		default:
			break;
		}
	}
	if (old_data->kickOffTeam != new_data->kickOffTeam) {
		new_info.messageNumber = KickOffTeam;
		new_info.extraData = uint16_t(new_data->kickOffTeam);
	}
	if (old_data->secondaryState != new_data->secondaryState) {
		switch (new_data->firstHalf)
		{
		case STATE2_NORMAL:
			new_info.messageNumber = RefereeInfo::State2_Normal;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE2_PENALTYSHOOT:
			new_info.messageNumber = RefereeInfo::State2_PenaltyShoot;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE2_OVERTIME:
			new_info.messageNumber = RefereeInfo::State2_Overtime;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE2_TIMEOUT:
			new_info.messageNumber = RefereeInfo::State2_Timeout;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE2_FREEKICK:
			new_info.messageNumber = RefereeInfo::State2_Freekick;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		case STATE2_PENALTYKICK:
			new_info.messageNumber = RefereeInfo::State2_PenaltyKick;
			new_info.extraData = NO_EXTRA_DATA;
			return true;
		default:
			break;
		}
	}
	if (old_data->dropInTeam != new_data->dropInTeam) {
		new_info.messageNumber = RefereeInfo::DropInTeam;
		new_info.extraData = uint16_t(new_data->dropInTeam);
		return true;
	}
	if (old_data->dropInTime != new_data->dropInTime) {
		new_info.messageNumber = RefereeInfo::DropInTime;
		new_info.extraData = new_data->dropInTime;
		return true;
	}
	if (old_data->teams[0].teamNumber == TEAM_NUMBER) {
		if (old_data->teams[0].players[PLAYER_NUMBER-1].penalty != new_data->teams[0].players[PLAYER_NUMBER-1].penalty) {
			uint16_t secsTillUnpenalised = uint16_t(new_data->teams[0].players[PLAYER_NUMBER].secsTillUnpenalised);
			cout<<(int)new_data->teams[0].players[PLAYER_NUMBER-1].penalty<<endl;
			switch (new_data->teams[0].players[PLAYER_NUMBER-1].penalty)
			{
			case HL_BALL_MANIPULATION:
				cout<<"HL_BALL_MANIPULATION"<<endl;
				new_info.messageNumber = RefereeInfo::Ball_Manipulation;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_PHYSICAL_CONTACT:
				cout<<"HL_PHYSICAL_CONTACT"<<endl;
				new_info.messageNumber = RefereeInfo::Physical_Contact;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_ILLEGAL_ATTACK:
				new_info.messageNumber = RefereeInfo::Illegal_Attack;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_ILLEGAL_DEFENSE:
				new_info.messageNumber = RefereeInfo::Illegal_Defense;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_PICKUP_OR_INCAPABLE:
				new_info.messageNumber = RefereeInfo::Pickup_Or_Incapable;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_SERVICE:
				new_info.messageNumber = RefereeInfo::Service;
				new_info.extraData = secsTillUnpenalised;
				return true;
			default:
				break;
			}
		}
	}
	else {
		if (old_data->teams[1].players[PLAYER_NUMBER-1].penalty != new_data->teams[1].players[PLAYER_NUMBER-1].penalty) {
			uint16_t secsTillUnpenalised = uint16_t(new_data->teams[0].players[PLAYER_NUMBER].secsTillUnpenalised);
			switch (new_data->teams[1].players[PLAYER_NUMBER-1].penalty)
			{
			case HL_BALL_MANIPULATION:
				cout<<"HL_BALL_MANIPULATION"<<endl;
				new_info.messageNumber = RefereeInfo::Ball_Manipulation;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_PHYSICAL_CONTACT:
				new_info.messageNumber = RefereeInfo::Physical_Contact;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_ILLEGAL_ATTACK:
				new_info.messageNumber = RefereeInfo::Illegal_Attack;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_ILLEGAL_DEFENSE:
				new_info.messageNumber = RefereeInfo::Illegal_Defense;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_PICKUP_OR_INCAPABLE:
				new_info.messageNumber = RefereeInfo::Pickup_Or_Incapable;
				new_info.extraData = secsTillUnpenalised;
				return true;
			case HL_SERVICE:
				new_info.messageNumber = RefereeInfo::Service;
				new_info.extraData = secsTillUnpenalised;
				return true;
			default:
				break;
			}
		}
	}
	return false;
}
