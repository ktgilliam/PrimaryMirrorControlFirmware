 /// 
 ///  @ Author: Kevin Gilliam
 ///  @ Create Time: 2022-09-06 12:05:48
 ///  @ Modified by: Kevin Gilliam
 ///  @ Modified time: 2022-09-07 15:30:36
 ///  @ Description:
 ///

#pragma once
#include <cstdint>

void initHeartbeat();
bool pingHeartBeat();
void resetHeartbeat();
void setHeartBeatPeriod(uint32_t cnts);
void toggleHeartbeatState();