/**
 * @file   VerifyOps.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Collection of macros to allow for checking whether operations are
 * allowed.
 */

#ifndef LUM_NET_COMMAND_VERIFY_OPS_H
#define LUM_NET_COMMAND_VERIFY_OPS_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

#define _LUM_NET_COMPARE_OP_AGAINST_OPS4(L_OT1, L_OST1, R_OT1, R_OST1)         \
  ((L_OT1 == R_OT1) && (L_OST1 == R_OST1))

#define _LUM_NET_COMPARE_OP_AGAINST_OPS6(L_OT1, L_OST1, R_OT1, R_OST1, R_OT2,  \
                                         R_OST2)                               \
  _LUM_NET_COMPARE_OP_AGAINST_OPS4(L_OT1, L_OST1, R_OT1, R_OST1) ||            \
      _LUM_NET_COMPARE_OP_AGAINST_OPS4(L_OT1, L_OST1, R_OT2, R_OST2)

#define _LUM_NET_COMPARE_OP_AGAINST_OPS8(L_OT1, L_OST1, R_OT1, R_OST1, R_OT2,  \
                                         R_OST2, R_OT3, R_OST3)                \
  _LUM_NET_COMPARE_OP_AGAINST_OPS6(L_OT1, L_OST1, R_OT1, R_OST1, R_OT2,        \
                                   R_OST2) ||                                  \
      _LUM_NET_COMPARE_OP_AGAINST_OPS4(L_OT1, L_OST1, R_OT3, R_OST3)

#define _LUM_NET_COMPARE_OP_AGAINST_OPS10(                                     \
    L_OT1, L_OST1, R_OT1, R_OST1, R_OT2, R_OST2, R_OT3, R_OST3, R_OT4, R_OST4) \
  _LUM_NET_COMPARE_OP_AGAINST_OPS8(L_OT1, L_OST1, R_OT1, R_OST1, R_OT2,        \
                                   R_OST2, R_OT3, R_OST3) ||                   \
      _LUM_NET_COMPARE_OP_AGAINST_OPS4(L_OT1, L_OST4, R_OT1, R_OST4)

#define _LUM_NET_VERIFY_VARDIAC_OP(...)                                        \
  (_LUM_NET_MAKE_VARIADIC_NAME(_LUM_NET_COMPARE_OP_AGAINST_OPS, __VA_ARGS__))

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Variable Declarations  */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Function Declarations  */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Function Definitions   */
/*********************************/
//@{

//@}

#endif // LUM_NET_COMMAND_VERIFY_OPS_H
