//-----------------------------------------------
// Neobotix: Linux-Version
// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author: Oliver Barth, Winfried Baum, Jens Kubacki, Christian Büsch
//-----------------------------------------------
#include <cob_forcetorque/CanSocket.h>
#include <string.h>
 
struct can_frame frame;
#define DEBUG 1

//-----------------------------------------------
CanSocket::CanSocket(const char* cIniFile, bool bObjectMode, int baudrate)
{
	m_bObjectMode = bObjectMode;
	m_bIsTXError = false;
	baudrate = 0x011C;
	initIntern(baudrate);
}

//-----------------------------------------------
 /*
 * Destructor.
 * Release the allocated resources.
 */
CanSocket::~CanSocket()
{
	std::cout << "Closing CAN handle" << std::endl;

}


//-----------------------------------------------
void CanSocket::initIntern(int bdrate)
{	
#ifdef DEBUG	
	std::cout << "begin init intern" << std::endl;
	std::cout << "m_skt is " << m_skt << std::endl;
#endif

	m_skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );
	
#ifdef DEBUG
	std::cout << "m_skt is " << m_skt << std::endl;
#endif

	/* Locate the interface you wish to use */
	struct ifreq ifr;
	strcpy(ifr.ifr_name, "can0");
	int ret = ioctl(m_skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled with that device's index */

#ifdef DEBUG
	std::cout << "error iocontrol" << ret << std::endl;
#endif	
	
	/* Select that CAN interface, and bind the socket to it. */
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	
#ifdef DEBUG	
	std::cout << "ifindex is " << addr.can_ifindex << std::endl;
#endif

	int ret1 = bind( m_skt, (struct sockaddr*)&addr, sizeof(addr) );

#ifdef DEBUG
	std::cout << "binding ok " << std::endl;
#endif

}

//-----------------------------------------------
/**
 * Transmit a message via the CAN bus.
 * Additionally, an error flag is set internally if the transmission does not succeed.
 * @param CMsg Structure containing the CAN message.
 * @return true on success, false on failure.
 */
bool CanSocket::transmitMsg(CanMsg CMsg, bool bBlocking)
{
	
	int bytes_sent =1;
	frame.can_id = CMsg.m_iID;
	
#ifdef DEBUG
	std::cout << "frame id is " << frame.can_id << std::endl;
#endif
	
	for(int i=0; i<8; i++)
	{

		frame.data[i] = CMsg.getAt(i);
		
	#ifdef DEBUG	
		std::cout << "frame.data " << i << "is: " << frame.data[i] << std::endl;
	#endif
	
	}

	frame.can_dlc = CMsg.m_iLen;
	
#ifdef DEBUG
	std::cout << "frame.can_dlc is: " << frame.can_dlc << std::endl;
#endif

	// copy string
	//frame.can_dlc = strlen( frame.data );
	//strcpy( frame.data, CMsg.data );
	//std::cout << "frame.data is: " << frame.data;
	//std::cout << "frame.can_dlc is: " << frame.can_dlc << std::endl;
	
	
#ifdef DEBUG
	std::cout << "m_skt is " << m_skt << std::endl;
#endif

	bool bRet = true;
	
	if (bBlocking) // blocking is not implemented yet!
		{
		bytes_sent = write( m_skt, &frame, sizeof(frame) );
		
	#ifdef DEBUG
		std::cout << "bblocking bytes sent " << bytes_sent << std::endl;
	#endif
		
		}
	else
		{bytes_sent = write( m_skt, &frame, sizeof(frame) );
		
	#ifdef DEBUG	
		std::cout << "not bblocking bytes sent " << bytes_sent << std::endl;
	#endif
		}
	if( bytes_sent > 0) 
	{
		
	#ifdef DEBUG	
		std::cout << "bytes sent in CanSocket::transmitMsg: " << bytes_sent << std::endl;
	#endif
	
		bRet = true;
	}
	
	if( bytes_sent < 0) 
	{
		std::cout << "error in CanSocket::transmitMsg: " << std::endl;
		bRet = false;
	}

	if (bytes_sent == 0) 
		bRet = false;

return bRet;
}

//-----------------------------------------------
bool CanSocket::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry) //not implemented yet
{
	bool bRet = false;
	return bRet;
}

//-----------------------------------------------
bool CanSocket::receiveMsg(CanMsg* pCMsg)
{
	
#ifdef DEBUG	
	std::cout << "receiving" << std::endl;
#endif

	bool bRet = true;
	frame.data[0] = 0;
	frame.data[1] = 0;
	frame.data[2] = 0;
	frame.data[3] = 0;
	frame.data[4] = 0;
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;
	frame.can_id = 0;
	frame.can_dlc = 0;
			
	int bytes_read = read( m_skt, &frame, sizeof(frame) );

#ifdef DEBUG	
	std::cout << "m_skt is " << m_skt << std::endl;
	std::cout << "bytes read " << bytes_read << std::endl;
	std::cout << "frame.can_id " << frame.can_id << std::endl;
	std::cout << "frame.can_dlc " << frame.can_dlc << std::endl;
#endif

	// if( !isObjectMode() ) //not implemented yet
	
		if (bytes_read > 0) 
		{
			// message received
			pCMsg->m_iID = frame.can_id;
			pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3],
				frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
			bRet = true;
		}
		else
		{
			// no message, error
			std::cout << "error in CanSocket::receiveMsg: " << std::endl;
			pCMsg->m_iID = frame.can_id;
			pCMsg->set(0,0,0,0,0,0,0,0);
			bRet = false;
		}
	
	return bRet;

}


/*!
    \fn CanESD::canIdAddGroup(m_Handle, int id, int number)
 */
/**
 * Add a group of CAN identifier to the handle, so it can be received.
 * The identifiers are generated by inverting the id and adding each value between 0 and 7
 * This is used for generating the answer commands by the RCS5000.
 * @param handle The handle to add the identifiers to.
 * @param id The command id sent to the RCS5000.
 * @return NTCAN_SUCESS if ok, or an error code.

int CanSocket::canIdAddGroup(HANDLE handle, int id)
{
	int result = NTCAN_SUCCESS;
	int i = 0;
	int iRes = 0;
	int cmd_id = invert(id);
	
	//m_Mutex.lock();
	for( i=0; i<8; ++i) {
		iRes = canIdAdd(m_Handle, cmd_id+i);
		//std::cout << "Adding CAN ID " << cmd_id+i << std::endl;
		if( iRes != NTCAN_SUCCESS ) {
			std::cout << "Adding CAN ID " << cmd_id+i << " failed with errorcode: " << iRes << " " << GetErrorStr(iRes) << std::endl;
			result = iRes;
		}
	}
	//m_Mutex.unlock();

	return result;
}
 */
//-----------------------------------------------
/*std::string CanSocket::GetErrorStr(int ntstatus) const
{
	switch (ntstatus)
	{
	case NTCAN_SUCCESS:			return "NTCAN_SUCCESS";
	case NTCAN_RX_TIMEOUT:			return "NTCAN_RX_TIMEOUT";
	case NTCAN_TX_TIMEOUT:			return "NTCAN_TX_TIMEOUT";
	case NTCAN_TX_ERROR:			return "NTCAN_TX_ERROR";
	case NTCAN_CONTR_OFF_BUS:		return "NTCAN_CONTR_OFF_BUS";
	case NTCAN_CONTR_BUSY:			return "NTCAN_CONTR_BUSY";
	case NTCAN_CONTR_WARN:			return "NTCAN_CONTR_WARN";
	case NTCAN_NO_ID_ENABLED:		return "NTCAN_NO_ID_ENABLED";
	case NTCAN_ID_ALREADY_ENABLED:		return "NTCAN_ID_ALREADY_ENABLED";
	case NTCAN_ID_NOT_ENABLED:		return "NTCAN_ID_NOT_ENABLED";

	case NTCAN_INVALID_FIRMWARE:		return "NTCAN_INVALID_FIRMWARE";
	case NTCAN_MESSAGE_LOST:		return "NTCAN_MESSAGE_LOST";
	case NTCAN_INVALID_HARDWARE:		return "NTCAN_INVALID_HARDWARE";

	case NTCAN_PENDING_WRITE:		return "NTCAN_PENDING_WRITE";
	case NTCAN_PENDING_READ:		return "NTCAN_PENDING_READ";
	case NTCAN_INVALID_DRIVER:		return "NTCAN_INVALID_DRIVER";

	case NTCAN_INVALID_PARAMETER:		return "NTCAN_INVALID_PARAMETER";
	case NTCAN_INVALID_HANDLE:		return "NTCAN_INVALID_HANDLE";
	case NTCAN_NET_NOT_FOUND:		return "NTCAN_NET_NOT_FOUND";
	case NTCAN_INSUFFICIENT_RESOURCES:	return "NTCAN_INSUFFICIENT_RESOURCES";
	
	case NTCAN_OPERATION_ABORTED:		return "NTCAN_OPERATION_ABORTED";
	}
	char msg[100];
	sprintf(msg, "unknown error code %d", ntstatus);
	return msg;
}
*/
/**
 * Check if errors occured on the CAN bus.
 * @return 	- 0 if everthing is fine.
 *		- -1 if an error occured.
 *		- -3 if messages were lost.
 *		- -5 if a FIFO overflow occured.
 *		- -6 if the CAN controller is BUS OFF.
 *		- -7 if the CAN controller is WARN, i.e. error passive.

int CanSocket::readEvent()
{
	EVMSG evmsg;
	int iRet = 0;
	int ret;

	//m_Mutex.lock();
	ret = canReadEvent(m_Handle, &evmsg, NULL);
	//m_Mutex.unlock();
	
	if(ret == NTCAN_SUCCESS)
	{
		if( (int)evmsg.evid == NTCAN_EV_CAN_ERROR ) {
			switch( evmsg.evdata.s[0] ) {
				case 0x00:
					iRet = 0;
					break;
				case 0xC0:
					iRet = -6;
					std::cout << "BUS OFF" << std::endl;
					break;
				case 0x40:
					iRet = -7;
					std::cout << "ERROR PASSIVE" << std::endl;
					break;
			}
			if( evmsg.evdata.s[3] != 0 ) {
				iRet = -3;
				std::cout << "Lost " << (int)evmsg.evdata.s[3] << " messages" << std::endl;
			} else if( evmsg.evdata.s[5] != 0 ) {
				iRet = -5;
				std::cout << "Lost " << (int)evmsg.evdata.s[5] << " messages from fifo" << std::endl;
			}
		}
	}
	return iRet;
}
 */
