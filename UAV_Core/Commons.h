#ifndef COMMONS_H
#define COMMONS_H

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <utilitiy/tools.h>

#ifdef __cplusplus
#define restrict __restrict__
#endif

#define UNUSED_ARG(x) ((void)x)

inline double Degree2Rad(double degree) {
  return degree * DEGTORAD;
}
//////////////////////////////////////////////////////////////////////////

inline double Rad2Degree(double rad) {
  return rad * RADTODEG;
}
//////////////////////////////////////////////////////////////////////////

inline void SwapPtrs(void **a, void **b) {
  void *tmp = *a;
  *a = *b;
  *b = tmp;
}
//////////////////////////////////////////////////////////////////////////

inline void SwapDoubles(double *a, double *b) {
  double tmp = *a;
  *a = *b;
  *b = tmp;
}
//////////////////////////////////////////////////////////////////////////

inline double MilesPerHour2MeterPerSecond(double mph) {
  return 2.23694 * mph;
}
//////////////////////////////////////////////////////////////////////////

inline int RandomBetween2Vals(int low, int hi) {
  assert(low <= hi);
  return (rand() % (hi - low)) + low;
}

inline double LowPassFilter(double prev, double measured, double alpha) {
  return prev + alpha * (measured - prev);
}


#endif // COMMONS_H
