

#include <stdbool.h>
#include <stdint.h>
#include "F28x_Project.h"
#include "math.h"
//#include "main2.h"

typedef struct Matrix
{
    float matr[3][3];
} Matrix;

typedef struct Vector
{
    float x, y, z;
} Vector;

typedef struct Point
{
    float alpha, beta;
    char flagStar;
}
Point;

typedef struct
{
    float careen;
    float tangage;
    float course;
} Angles;

unsigned char var0, var1, var2, var3;
long var, varVar;

double binsXAngle = 0, initialBinsXAngle = 0;
double binsYAngle = 0, initialBinsYAngle = 0;
double binsZAngle = 0, initialBinsZAngle = 0;



unsigned long longVar = 0;
int numberOfValues = 10;
long sumValue = 65535 * 4;


Angles memsAngles = { 0, 0, 0 };
Angles binsAngles = { 0, 0, 0 };

long renishaw1 = 0;
long renishaw2 = 0;

Vector vector = { 0, 0, 0 };
Vector newVector = { 0, 0, 0 };

Point point;
Point newPoint;

Matrix matrix;

char stabilizationFlag = 0;

// 1 - mems
// 2 - bins

#define MEMS 1
#define BINS 2
char stabilizationMode = MEMS;

int positionArray[1000];
unsigned long positionCounter = 0;
long frequencyTimer = 0;

/*
    Матрица перехода с учётом курса, крена, тангажа
    theta - тангаж
    psi - курс
    gamma - крен

 */
Matrix GetTransitionMatrix(float theta,float psi,float gamma)
{
    Matrix tmpMatrix;

    tmpMatrix.matr[0][0] = cos(theta) * cos(psi);
    tmpMatrix.matr[0][1] = sin(theta);
    tmpMatrix.matr[0][2] = cos(theta) * sin(psi);

    tmpMatrix.matr[1][0] = - cos(gamma) * sin(theta) * cos(psi) - sin(gamma) * sin(psi);
    tmpMatrix.matr[1][1] = cos(theta) * cos(gamma);
    tmpMatrix.matr[1][2] = - sin(theta) * sin(psi) * cos(gamma) + cos(psi) * sin(gamma);

    tmpMatrix.matr[2][0] = sin(theta) * cos(psi) * sin(gamma) - sin(psi) * cos(gamma);
    tmpMatrix.matr[2][1] = - cos(theta) * sin(gamma);
    tmpMatrix.matr[2][2] = sin(theta) * sin(psi) * sin(gamma) + cos(psi) * cos(gamma);


    return tmpMatrix;

}



//@@@
// Из сферич в прямоуг
Vector SphereToRectangle(Point a)
{
    Vector result;
    result.x = cos(a.beta)*cos(a.alpha);
    result.y = sin(a.beta);
    result.z = cos(a.beta)*sin(a.alpha);
    return result;
}

//@@@
//----------------------------Умножение матрицы на вектор-столбец  ----------------------------------------------//

Vector MultiplyMatrixes(struct Matrix a, struct Vector b)
{
    float d [3];
    float c [3];
    c[0] = b.x;
    c[1] = b.y;
    c[2] = b.z;
    int i, j;
    for (i = 0; i < 3; i++)
    {
        d[i]=0;
        for (j = 0; j < 3; j++)
        {
            d[i]+= a.matr[i][j]*c[j];
        }
    }
    Vector result;
    result.x = d[0];
    result.y = d[1];
    result.z = d[2];
    return result;
}


