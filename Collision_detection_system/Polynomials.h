#pragma once

#include "Constants.h"
#include <complex>
#include <cmath>


// y = ax^2 + bx + c
// roots is assumed to be at least of length 2
// returns number of roots
int polynomial2Roots(float a, float b, float c, float* roots);


// y = ax^3 + bx^2 + cx + d
// roots is assumed to be at least of length 3
// returns number of roots
int polynomial3Roots(float a3, float a2, float a1, float a0, float* roots);



// y = a4x^4 + a3x^3 + a2x^2 + a1x + a0
// roots is assumed to be at least of length 4
// returns number of roots
int polynomial4Roots(float a4, float a3, float a2, float a1, float a0, float* roots);

