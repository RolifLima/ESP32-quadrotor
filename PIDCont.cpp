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

#include "Arduino.h"
#include "PIDCont.h"

PIDCont::PIDCont()
{
float pError = 0.0;
float Ip = 0.000;
float kp=0.000;
float ki=0.000;
float kd=0.000;
float Hval=0;
float Lval=0;
}

void PIDCont::SetParams(float mKp, float mKi, float mKd, float mLval, float mHval)
{

kp=mKp;
ki=mKi;
kd=mKd;
Lval = mLval;
Hval = mHval;
}

float PIDCont::Compute(float mError,float dt)//dt in milli sec
{ float P =  kp * mError;
  float D =  (kd * (mError-pError) * 1000.0/dt);
  pError = mError;
  float I =  (Ip + ki * mError * dt /1000.0);
  float U =  (P + I + D);
  Ip = I;

  if(U>Hval){
    U = Hval;
  }
  else if (U<Lval){
    U = Lval;
  }
return U;

}

float PIDCont::Compute(float mError,float dError,float dt)
{
  float P = (float) kp * mError;
  float D = (float) (kd *1000/dt*dError);
  float I = (float) (Ip + ki * mError * dt /1000.0);
  float U = (float) (P + I + D);
  pError = mError;
  Ip = I;
  if(U>Hval){
    U = Hval;
  }
  else if (U<Lval){
    U = Lval;
  }
return U;

}

void PIDCont::resetI()
{
Ip=0.0;
tp=millis();
}
