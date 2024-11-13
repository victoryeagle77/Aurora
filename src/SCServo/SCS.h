﻿#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS{
public:
    SCS();
    SCS(u8 End);
    SCS(u8 End, u8 Level);

    int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
    int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
    int RegWriteAction(u8 ID = 0xfe);
    void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);
    int writeByte(u8 ID, u8 MemAddr, u8 bDat);
    int writeWord(u8 ID, u8 MemAddr, u16 wDat);
    int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);
    int readByte(u8 ID, u8 MemAddr);
    int readWord(u8 ID, u8 MemAddr);
    int Ping(u8 ID);
    int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);
    int syncReadPacketRx(u8 ID, u8 *nDat);
    int syncReadRxPacketToByte();
    int syncReadRxPacketToWrod(u8 negBit=0);

public:
    u8 Level, End, Error;
    u8 syncReadRxPacketIndex, syncReadRxPacketLen;
    u8 *syncReadRxPacket;

protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;

protected:
    void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
    void Host2SCS(u8 *DataL, u8* DataH, u16 Data);
    u16 SCS2Host(u8 DataL, u8 DataH);
    int Ack(u8 ID);
    int checkHead();
};

#endif
