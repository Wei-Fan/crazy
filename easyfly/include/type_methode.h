#ifndef TYPE_METHODS_H
#define TYPE_METHODS_H
#include <math.h>
#include "basic_types.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
using namespace Eigen;

#define _USE_MATH_DEFINES //PI

void body2earth(const Matrix3f* R, const Vector3f* body, Vector3f* earth, short dimension);

void earth2body(const Matrix3f* R, const Vector3f* earth, Vector3f* body, short dimension);//body=inv(R)*earth

void rotation2euler(const Matrix3f* R, Vector3f* Euler);

void euler2rotation(const Vector3f* Euler, Matrix3f* R);

float data_2_angle(float x, float y, float z);	 //in rad

void quaternion2rotation(const Vector4f* Q, Matrix3f* R);

void rotation2quaternion(const Matrix3f* R, Vector4f* Q);

void euler2quaternion(const Vector3f* Euler, Vector4f* Q);

void vec3f_norm(const Vector3f* a, float* anwser);

//void quaternion_derivative(const Vector4f* Q, Vector4f* derQ, const Vector3f* w);

float deriv_f(const	float f_now, const float f_past , const float dt);

void quaternion_normalize(Vector4f* Q);


void vec3f_normalize(Vector3f* v);

void vec3f_passnorm(const Vector3f* v, Vector3f* vec_des);

float vec3f_length(const Vector3f* v);

float vec3f_dot(const Vector3f* a, const Vector3f* b);

void vec3f_cross(const Vector3f* a, const Vector3f* b, Vector3f* d);

void vec3f_integration(Vector3f* Integrated, Vector3f* Origin, float dt);

void vec3f_derivative(Vector3f* Deriv, Vector3f* Origin, Vector3f* l_Origin, float dt);

//void add_A2B(const Vector3f* a, Vector3f*b);

float degToRad(float deg);

float radToDeg(float deg);

void writeData_bin(const char* fname, Vector3f* vec);

void writeData_binf(const char* fname, float number);
#endif
