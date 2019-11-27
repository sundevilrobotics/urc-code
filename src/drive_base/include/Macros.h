#ifndef MACROS_H
#define MACROS_H

#define CLAMP2(value,lower,upper) value = value < lower ? lower : value > upper ? upper : value;
#define CLAMP(value, clampval) CLAMP2(value,-clampval,clampval)

#endif