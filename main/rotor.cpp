// Copyright (C) 2018 kaz Kojima
//
// This file is part of ef/bee3d program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the file COPYING.

#include <cstdint>
#include <cstdio>

#define USE_3RD_ORDER_INTERPORATION 1
//#define USE_TRAPEZOIDAL_INTERPORATION 1
#include "rotor.h"

/*
  This is an experimental standalone implementation of a 3D rotor
  idendification with geometric algebra.  GA computations are
  hand-compiled to a few arithmetics of Euclidean vector and bivector.

  Some common naming rule for variables:
  Postfix x (resp. y, z) corresponds the standard 3D Euclidean base
  e1 (resp. e2, e3) which is a unit vector on x(resp. y, z)-axis.
  Postfix i (resp. j, k) represents the unit bivector e2^e3 (resp.
  e3^e1, e1^e2), i.e. can be identified with the pure quatanion i
  (resp. j, k).
*/

/*
  RotorIy: Base class.
*/

RotorIy::RotorIy ()
{
  m_S0 = 1.0f;
  m_Si = m_Sj = m_Sk = 0.0f;
}

RotorIy::~RotorIy ()
{
}

// v = S*(-e3)/S
// -2 q0 q2 - 2 q1 q3, 2 q0 q1 - 2 q2 q3, -q0^2 + q1^2 + q2^2 - q3^2
inline static void
apply_me3 (float c, float si, float sj, float sk,
	   float& vx, float& vy, float& vz)
{
  vx = -2*c*sj - 2*si*sk;
  vy =  2*c*si - 2*sj*sk;
  vz = -c*c + si*si + sj*sj - sk*sk;
}

void
RotorIy::Show (void)
{
  //printf ("S=%2.3f %2.3f %2.3f %2.3f\n", m_S0, m_Si, m_Sj, m_Sk);
  printf ("S=%2.3f+(%2.3f)*e2^e3+(%2.3f)*e3^e1+(%2.3f)*e1^e2;$\n", m_S0, m_Si, m_Sj, m_Sk);
  //float vx, vy, vz;
  //apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
  //printf ("%2.6f %2.6f %2.6f\n", vx, vy, vz);
}

/*
  Common additional fast finite float math functions.
*/

#if !defined(USE_LUT_SINCOS)
#include <cmath>
#else
extern "C" float Sinf(float x);
extern "C" float Cosf(float x);
#define sinf Sinf
#define cosf Cosf
#define FIXUP_NORM
#endif

static const float float_ep = 1.192093e-07;
static const float root_ep = 3.452670e-04;
static const float root_root_ep = 1.858136e-02;
static const float float_pi = 3.14159265359;

static float
invSqrt (float x)
{
  union { int32_t i; float f; } u;
  float halfx = 0.5f * x;
  float y = x;
  u.f = y;
  u.i = 0x5f3759df - (u.i>>1);
  y = u.f;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static float
Sqrt (float x)
{
  return __builtin_sqrtf (x);
}

static float
Sincf(float x)
{
  if (x < 0.0f)
    x = -x;
  if (x >= root_root_ep)
    return sinf(x)/x;
  // Approx. near by 0
  float sc = 1.0f;
  if (x >= float_ep)
    {
      float x2 = x*x;
      sc -= x2/6;
      if (x >= root_ep)
	sc += x2*x2/120;
    }

  return sc;
}
  
static float
Atan2 (float y, float x)
{
  float absx = x >= 0.0f ? x : -x;
  float absy = y >= 0.0f ? y : -y;
  float absmin = absx >= absy ? absy : absx;
  float absmax = absx >= absy ? absx : absy;
  float a = absmin/absmax;
  float s = a*a;
  // 7th order Taylor approximation
  float r = ((-0.0464964749f*s + 0.15931422f)*s - 0.327622764f)*s*a + a;
  if (absy > absx)
    r = float_pi/2 - r;
  if (x < 0.0f)
    r = float_pi - r;
  if (y < 0.0f)
    r = -r;
  return r;
}

// constants
static const float sq_gravity_mss = (9.80665f*9.80665f);
static const float rotor_ep = 1.0e-6;

/*
  RotorIyVS: Implementation with versor(rotor).
*/

RotorIyVS::RotorIyVS ()
{
  m_Ii = m_Ij = m_Ik = 0.0f;
  m_omega0i = m_omega0j = m_omega0k = 0.0f;
  m_omega1i = m_omega1j = m_omega1k = 0.0f;
  m_gain = 0.0f;
}

RotorIyVS::RotorIyVS (float gain, float dt, float epsilon):
  RotorIy(), m_gain(gain), m_dt(dt), m_epsilon(epsilon)
{
  m_omega0i = m_omega0j = m_omega0k = 0.0f;
  m_omega1i = m_omega1j = m_omega1k = 0.0f;
  m_Ii = m_Ij = m_Ik = 0.0f;
}

RotorIyVS::~RotorIyVS ()
{
}

#if defined(USE_3RD_ORDER_INTERPORATION)
// inline version of commutator product for Euclidean bivectors.
// Return (1/2.0) (x*y - y*x) where * is geometric product.
inline static void
comm (const float a, const float b, const float c,
      const float d, const float e, const float f,
      float& x, float& y, float& z)
{
  x = -b*f + c*e;
  y =  a*f - c*d;
  z = -a*e + b*d;
}
#endif

void
RotorIyVS::Update (float gi, float gj, float gk,
		   float ax, float ay, float az,
		   float& c, float& si, float& sj, float& sk,
		   float& omegai, float& omegaj, float& omegak)
{
  // Wash out gyro drifts
  m_Ii = (1 - m_epsilon)*m_Ii + m_epsilon*gi;
  m_Ij = (1 - m_epsilon)*m_Ij + m_epsilon*gj;
  m_Ik = (1 - m_epsilon)*m_Ik + m_epsilon*gk;
  omegai = gi - m_Ii;
  omegaj = gj - m_Ij;
  omegak = gk - m_Ik;

  if (m_gain > 0)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      float nm2 = (ax+vx)*(ax+vx)+(ay+vy)*(ay+vy)+(az+vz)*(az+vz);
      float nm3 = ax*ax + ay*ay + az*az;
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      if (nm2 > m_norm_threshold
	  && 0.8f*sq_gravity_mss < nm3 && nm3 < 1.2f*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  // Skip if nm is too small
	  if (nm > float_ep)
	    {
	      float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	      omegai -= ac*pi;
	      omegaj -= ac*pj;
	      omegak -= ac*pk;
	    }
	}
    }

#if defined(USE_3RD_ORDER_INTERPORATION)
  // 3rd order approximation by Candy and Lasenby.
  // _omega0 = \omega(-T), _omega1 = \omega(0), omega = \omega(T)
  float ci, cj, ck;
  comm (omegai - m_omega0i, omegaj - m_omega0j, omegak - m_omega0k,
	m_omega1i, m_omega1j, m_omega1k,
	ci, cj, ck);

  omegai = ((1.0f/12)*(-m_omega0i + 8.0f*m_omega1i + 5.0f*omegai)
	    + (1.0f/24)*m_dt*ci);
  omegaj = ((1.0f/12)*(-m_omega0j + 8.0f*m_omega1j + 5.0f*omegaj)
	    + (1.0f/24)*m_dt*cj);
  omegak = ((1.0f/12)*(-m_omega0k + 8.0f*m_omega1k + 5.0f*omegak)
	    + (1.0f/24)*m_dt*ck);

  m_omega0i = m_omega1i;
  m_omega0j = m_omega1j;
  m_omega0k = m_omega1k;
  m_omega1i = omegai;
  m_omega1j = omegaj;
  m_omega1k = omegak;
#elif defined(USE_TRAPEZOIDAL_INTERPORATION)
  // Trapezoidal interpolation
  m_omega1i = omegai;
  m_omega1j = omegaj;
  m_omega1k = omegak;

  omegai = 0.5f*(omegai + m_omega1i);
  omegaj = 0.5f*(omegaj + m_omega1j);
  omegak = 0.5f*(omegak + m_omega1k);
#endif

  float delta = 0.5f*m_dt*Sqrt(omegai*omegai + omegaj*omegaj + omegak*omegak);

  float dc = cosf(delta);
  float dsc = -0.5f*m_dt*Sincf(delta);
  float dsi = dsc*omegai;
  float dsj = dsc*omegaj;
  float dsk = dsc*omegak;
  c = dc*m_S0 - dsi*m_Si - dsj*m_Sj - dsk*m_Sk;
  si = dsi*m_S0 + dc*m_Si - dsk*m_Sj + dsj*m_Sk;
  sj = dsj*m_S0 + dsk*m_Si + dc*m_Sj - dsi*m_Sk;
  sk = dsk*m_S0 - dsj*m_Si + dsi*m_Sj + dc*m_Sk;
#if defined(FIXUP_NORM)
  float inm = invSqrt (c*c + si*si + sj*sj + sk*sk);
  c *= inm; si *= inm; sj *= inm; sk *= inm;
#endif
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
}

void
RotorIyVS::SetGain (float gain)
{
  if (gain >= 0.0f)
    m_gain = gain;
}
