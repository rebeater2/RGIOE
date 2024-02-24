/******************************************************************************
 * RGIOE:A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based on Extend Kalman Filter
 * Copyright (C) 2024                                                         *
 * Author : rebeater                                                          *
 * Contact : rebeater@qq.com                                                  *
 ******************************************************************************/

#ifndef RGIOE_MACROLIB_H
#define RGIOE_MACROLIB_H
/* Compiler */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 5000000) /* ARM Compiler \
                                                              */
#define RGIOE_WEAK_FUNC __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__) /* for IAR Compiler */
#define RGIOE_WEAK_FUNC __weak
#elif defined(__MINGW32__) /* MINGW32 Compiler */
#define RGIOE_WEAK_FUNC __attribute__((weak))
#elif defined(__GNUC__) /* GNU GCC Compiler */
#define RGIOE_WEAK_FUNC __attribute__((weak))
#endif
/* default RGIOE_WEAK_FUNC */
#ifndef RGIOE_WEAK_FUNC
#define RGIOE_WEAK_FUNC
#endif

#endif //RGIOE_MACROLIB_H
