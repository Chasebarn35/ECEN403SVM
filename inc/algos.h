#ifndef __SVMALGOS
#define __SVMALGOS


/* PHASE SWITCH SETTING FUNCTION
 *
 * Given a 6 bit int, set the phase inputs to outputs
 * 00 -> Turn Phase OFF
 * 01 -> Turn Phase to a
 * 10 -> Turn Phase to b
 * 11 -> Turn Phase to c
 * Instructions is little endian (probably)
 * EXAMPLE:
 * 0b001001 -> input phase C OFF, input phase B set to b, input phase A set to a
 */
void PhaseChange(uint8_t Phases);



#endif
