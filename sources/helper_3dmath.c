/*
 * helper_3dmath.c
 *
 * Created: 2015-11-23 20:14:48
 * Author: remy mourard
 * Note: this is a C port of JRowberg I2C devlib
 */
#include "helper_3dmath.h"
#include <math.h>

void initQuaternion(Quaternion* aQuaternion, float aW, float aX, float aY, float aZ)
{
	aQuaternion->w = aW;
	aQuaternion->x = aX;
	aQuaternion->y = aY;
	aQuaternion->z = aZ;
}

void computeQuaternionProduct(Quaternion* aQuaternion, float aW, float aX, float aY, float aZ)
{
    float currentW = aQuaternion->w;
    float currentX = aQuaternion->x;
    float currentY = aQuaternion->y;
    float currentZ = aQuaternion->z;

    aQuaternion->w = currentW*aW - currentX*aX - currentY*aY - currentZ*aZ;
	aQuaternion->x = currentW*aX + currentX*aW + currentY*aZ - currentZ*aY;
	aQuaternion->y = currentW*aY - currentX*aZ + currentY*aW + currentZ*aX;
	aQuaternion->z = currentW*aZ + currentX*aY - currentY*aX + currentZ*aW;
}

void computeQuaternionConjugate(Quaternion* aQuaternion, Quaternion* aConjugateQuaternion)
{
    initQuaternion(aConjugateQuaternion, aQuaternion->w, -aQuaternion->x, -aQuaternion->y, -aQuaternion->z);
}

float getQuaternionMagnitude(Quaternion* aQuaternion)
{
	return sqrtf(powf(aQuaternion->w, 2) + powf(aQuaternion->x, 2) + powf(aQuaternion->y, 2) + powf(aQuaternion->z, 2));
}

void normalizeQuaternion(Quaternion* aQuaternion)
{
    float magnitude = getQuaternionMagnitude(aQuaternion);
	aQuaternion->w = aQuaternion->w / magnitude;
    aQuaternion->x = aQuaternion->x / magnitude;
	aQuaternion->y = aQuaternion->y / magnitude;
	aQuaternion->z = aQuaternion->z / magnitude;
}

void initVectorFloat(VectorFloat* aVector, float aX, float aY, float aZ)
{
    aVector->x = aX;
    aVector->y = aY;
    aVector->z = aZ;
}

float getVectorMagnitude(VectorFloat* aVector)
{
    return sqrtf(powf(aVector->x,2)+powf(aVector->y,2)+powf(aVector->z,2));
}

void normalizeVector(VectorFloat* aVector)
{
    float mag = getVectorMagnitude(aVector);
    aVector->x = aVector->x / mag;
    aVector->y = aVector->y / mag;
    aVector->z = aVector->z / mag;
}

void rotate(VectorFloat* aVector, Quaternion* toConjugateQuaternion)
{
    // create quaternion from vector values
    Quaternion p, qConj;
    initQuaternion(&p, 0, aVector->x, aVector->y, aVector->z);

    computeQuaternionConjugate(toConjugateQuaternion, &qConj);

    // quaternion multiplication: q * p, q pointer modified in void
    computeQuaternionProduct(toConjugateQuaternion, p.w, p.x, p.y, p.z);
    // stored back in p
    p = *toConjugateQuaternion;

    // quaternion multiplication: p * conj(q), p pointer modified in void
    computeQuaternionProduct(&p, qConj.w, qConj.x, qConj.y, qConj.z);

    aVector->x = p.x;
    aVector->y = p.y;
    aVector->z = p.z;
}