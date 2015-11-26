/*
 * helper_3dmath.h
 *
 * Created on: Oct 27, 2015
 * Author: remy mourard
 * note this code is a port of JRowberg I2C Devlib, credit to him so
 */

#ifndef HEADERS_HELPER_3DMATH_H_
#define HEADERS_HELPER_3DMATH_H_
#include <math.h>

/************************************************************************/
/* @brief Quaternion struct for holding Quaternions data                */
/************************************************************************/
typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;


/************************************************************************/
/* @brief helper to init a new quaternion by values                     */
/************************************************************************/
void initQuaternion(Quaternion* aQuaternion, float aW, float aX, float aY, float aZ);

/************************************************************************/
/* @brief Quaternion multiplication is defined by:                      */
/*        (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)                     */
/*        (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)                     */
/*        (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)                     */
/*        (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2                      */
/************************************************************************/
void computeQuaternionProduct(Quaternion* aQuaternion, float aW, float aX, float aY, float aZ);

/************************************************************************/
/* @brief return a pointer to the conjugate of a given Quaternion       */
/*                                                                      */
/************************************************************************/
void computeQuaternionConjugate(Quaternion* aQuaternion, Quaternion* aConjugateQuaternion);

/************************************************************************/
/* @brief compute magnitude                                             */
/************************************************************************/
float getQuaternionMagnitude(Quaternion* aQuaternion);

/************************************************************************/
/* @brief normalize quaternion like you normalize a vector              */
/************************************************************************/
void normalizeQuaternion(Quaternion* aQuaternion);

/************************************************************************/
/* @brief VectorFloat struct, a 3D vector with float member             */
/************************************************************************/
typedef struct {
    float x;
    float y;
    float z;
} VectorFloat;

/************************************************************************/
/* @brief create a new vector and returns it                            */
/************************************************************************/
void initVectorFloat(VectorFloat* aVector, float aX, float aY, float aZ);

/************************************************************************/
/* @brief compute magnitude                                             */
/************************************************************************/
float getVectorMagnitude(VectorFloat* aVector);

/************************************************************************/
/* @brief normalize vector                                              */
/************************************************************************/
void normalizeVector(VectorFloat* aVector);

/************************************************************************/
/* @brief compute rotation for geting y/p/r angles                      */
/************************************************************************/
void rotate(VectorFloat* aVector, Quaternion* toConjugateQuaternion);

#endif /* HEADERS_HELPER_3DMATH_H_ */
