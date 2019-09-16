/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年4月1日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <bitset>
using namespace std;

//使用bitset转换二进制
void BinaryBitset(short n)
{
  cout<<bitset<sizeof(short)*8>(n)<<endl;
}

int main()
{
  signed short a =7;
  cout<<a<<endl;
  BinaryBitset(a);
  signed short b = a<<2;
  BinaryBitset(b);
  BinaryBitset(a);

  unsigned short c = 0b1011100110110000;
  unsigned short d = (~c) + 1;
  unsigned short e = 0x0FFFF&d;
  cout<<e<<endl;

  cout<<10/1e-7<<endl;
}
