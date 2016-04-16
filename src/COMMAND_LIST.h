/*
 * COMMAND_LIST.h
 *
 *  Created on: Feb 21, 2016
 *      Author: CornellCup
 */

#ifndef COMMAND_LIST_H_
#define COMMAND_LIST_H_

#define AUTOBAUD				0xAA
#define CMD_START				32
#define CMD_UNITS				33
#define CMD_HOME				34
#define CMD_MOVE				36
#define CMD_GET					35
#define CMD_REPLY				67
#define CMD_SYSTEM      37

#define GET_ERR_NOERROR       0
#define GET_ERR_CHNOTSTARTED	1
#define GET_ERR_CHNEEDSHOMED	2
#define GET_ERR_CNTRLERROR		3
#define GET_ERR_WRONGMODE     4
#define GET_ERR_PARAMUNKNOWN	5
#define GET_ERR_SERTIMEOUT		6

#define SYS_CMD_PWRDOWN       0

#define MUX_COMPUTER			1
#define MUX_KANGAROO			0

#endif /* COMMAND_LIST_H_ */
