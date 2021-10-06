/**
 * @file   ShiftAndMask.h
 * @author Istvan Burbank
 * @date   2018
 * @brief
 */

#ifndef LUM_NET_SHIFT_AND_MASK_H
#define LUM_NET_SHIFT_AND_MASK_H

/*********************************/
/* Includes                      */
/*********************************/
/* Includes that anyone who includes this header needs                        */
//@{

#include <stdint.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
/* Naming Examples:                                                           */
/* - #define EXAMPLE_TEMPLATE_VERSION_NUMBER (1)                              */
/* - #define EXAMPLE_TEMPLATE_FORMAT(a)      (a)                              */
//@{

/**
 * Given a start and length output a mask which zeros any bits not associated
 * with that range.
 *
 * The C spec says shifting a word by the number of bits in the word is
 * undefined behavior which is why I put the casting in this function.
 */
#define LUM_NET_SHIFT_AND_MASK_BIT_RANGE_MASK(start, length)                   \
  (uint32_t)((~((uint64_t) 0) << (start)) ^                                    \
             (~((uint64_t) 0) << (start + length)))

/**
 * Given a start, length, and a `uint32_t input_word` get variable specified
 * by the range from the `input_word`.
 */
#define LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(start, length, input_word)     \
  (((input_word) &LUM_NET_SHIFT_AND_MASK_BIT_RANGE_MASK(start, length)) >>     \
   start)

/**
 * Given a start, length, a `uint32_t input_word`, and a variable, this
 * will set the variable specified by the range in the `input_word`.
 */
#define LUM_NET_SHIFT_AND_MASK_BIT_RANGE_SETTER(input_word, start, length,     \
                                                value)                         \
  (((input_word) & ~LUM_NET_SHIFT_AND_MASK_BIT_RANGE_MASK(start, length)) |    \
   (value) << start)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
/* Don't typedef structs/unions                                               */
/* Naming Examples:                                                           */
/* - union   Template_parameterOverlay {...};                                 */
/* - typedef unsigned int ExampleTemplate_columnIndex;                        */
/* - enum Template_itemType {TEMPLATE_ITEM_TYPE_A, TEMPLATE_ITEM_TYPE_B};     */
//@{

//@}

/*********************************/
/* GLOBAL Variable Declarations  */
/*********************************/
/* Declare as extern                                                          */
/* Naming Examples:                                                           */
/* - extern int ExampleTemplate_variableName;                                 */
//@{

//@}

/*********************************/
/* GLOBAL Function Declarations  */
/*********************************/
/* Don't declare as extern                                                    */
/* Naming Examples:                                                           */
/* - void ExampleTemplate_Init( void );                                       */
/* - void Template_ReturnZero( void );                                        */
//@{

//@}

/*********************************/
/* GLOBAL Function Definitions   */
/*********************************/
/* Declare as inline                                                          */
/* These should only be small, performance-critical functions in the header   */
//@{

//@}

#endif // LUM_NET_SHIFT_AND_MASK_H
