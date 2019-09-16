/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月18日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/
#ifndef _TYPE_DEFINE_
#define _TYPE_DEFINE_
#include <iostream>

enum TrackState
{
  TS_NONE = 0,
  Tentative,
  Confirmed,
  Deleted
};

class Base
{
public:
  Base(){};
};

#endif
