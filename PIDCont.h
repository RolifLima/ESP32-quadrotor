/*
  Created by Basel Al-Rudainy, 6 april 2013.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/
#ifndef PIDCont_h
#define PIDCont_h

#include "Arduino.h"

class PIDCont
{
public:
PIDCont();
void SetParams(float mKp, float mKi, float mKd, float mLval, float mHval);
float Compute(float mError,float dt);
float Compute(float mError,float dError,float dt);
void resetI();

private:
float kp;
float ki;
float kd;
float pError;
float Ip;
float Lval;
float Hval;
unsigned long tp;
};

#endif
