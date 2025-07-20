#ifndef __DEFINE_FILE_H
#define __DEFINE_FILE_H
//定义一些define的量方便使用
#include "string.h"
#include "stdint.h"
#define byte uint8_t
#define FOR(i,a,b) for(byte i=(a);i<(b);i++)
#define REP(i,a)  FOR(i,0,a)
#define ZERO(m) memset(m,0,sizeof(m))
#define SET_VALUE(m,a) memset(m,a,sizeof(m))
#endif 
