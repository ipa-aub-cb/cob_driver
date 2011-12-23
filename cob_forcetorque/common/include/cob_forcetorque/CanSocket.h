/* -----------------------------------------------------------------------------------
 *
 *		Copyright (c) 2005 Neobotix (www.neobotix.de)
 *
 *      This software is allowed to be used and modified only in association with a Neobotix
 *      robot platform. It is allowed to include the software into applications and
 *      to distribute it with a Neobotix robot platform. 
 *      It is not allowed to distribute the software without a Neobotix robot platform. 
 *
 *      This software is provided WITHOUT ANY WARRANTY; without even the
 *		implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *		PURPOSE. See the Neobotix License (Version 1.0) for more details.
 *
 *      You should have received a copy of the Neobotix License
 *      along with this software; if not, write to the 
 *      Gesellschaft fuer Produktionssysteme, Neobotix, Nobelstrasse 12, 70569 Stuttgart, Germany
 *
 * -----------------------------------------------------------------------------------
 */


#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
 
#ifndef PF_CAN
#define PF_CAN 29
#endif
 
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#ifndef CANSOCKET_INCLUDEDEF_H
#define CANSOCKET_INCLUDEDEF_H

//-----------------------------------------------
#include <iostream>
#include <cstdio>
#include <errno.h>
#include <cob_forcetorque/CanItf.h>
#include <cob_forcetorque/Mutex.h>

//-----------------------------------------------
/**
 * Driver of the CAN controller of ESD.
 * \ingroup LinuxDriversCanModul
 */
class CanSocket : public CanItf
{
private:
	BYTE m_DeviceNr;
	BYTE m_BaudRate;
	int m_Handle;
	int m_LastID;
	bool m_bObjectMode;
	bool m_bIsTXError;
	Mutex m_Mutex;
	int m_skt;
	void initIntern(int bdrate);
	

public:
	CanSocket(const char* cIniFile, bool bObjectMode = false, int baudrate = 0);
	~CanSocket();
	void init(){};
	bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
	bool receiveMsg(CanMsg* pCMsg);
	bool isObjectMode() { return m_bObjectMode; }
	bool isTransmitError() { return m_bIsTXError; }
protected:

    
	int invert(int id)
	{
	//return (~id) & 0x7FF;
		return (~id) & 0x7F8;
	}
	
	int canIdAddGroup(int handle, int id);

	std::string GetErrorStr(int ntstatus) const;
	int readEvent();
};
//-----------------------------------------------
#endif
