#include "Matrix3.h"


Matrix3::Matrix3() :
	m11(0.0f), m12(0.0f), m13(0.0f),
	m21(0.0f), m22(0.0f), m23(0.0f),
	m31(0.0f), m32(0.0f), m33(0.0f)
{};

Matrix3::Matrix3(float _m11, float _m12, float _m13, float _m21, float _m22, float _m23,float _m31, float _m32, float _m33) :
        m11(_m11), m12(_m12), m13(_m13),
        m21(_m21), m22(_m22), m23(_m23),
        m31(_m31), m32(_m32), m33(_m33)
{};

Matrix3::Matrix3(const Matrix3& _m) :
        m11(_m.m11), m12(_m.m12), m13(_m.m13),
        m21(_m.m21), m22(_m.m22), m23(_m.m23),
        m31(_m.m31), m32(_m.m32), m33(_m.m33)
{};

Matrix3& Matrix3::operator= (const Matrix3& _m)
{
    std::memcpy(m, _m.m, sizeof(m));

    /*
    m11 = _m.m11;
    m12 = _m.m12;
    m13 = _m.m13;
    m21 = _m.m21;
    m22 = _m.m22;
    m23 = _m.m23;
    m31 = _m.m31;
    m32 = _m.m32;
    m33 = _m.m33;
    */

    return *this;
};

Matrix3 Matrix3::operator* (const Matrix3& _m) const
{
    return Matrix3(
            m11 * _m.m11 + m12 * _m.m21 + m13 * _m.m31,
            m11 * _m.m12 + m12 * _m.m22 + m13 * _m.m32,
            m11 * _m.m13 + m12 * _m.m23 + m13 * _m.m33,
            m21 * _m.m11 + m22 * _m.m21 + m23 * _m.m31,
            m21 * _m.m12 + m22 * _m.m22 + m23 * _m.m32,
            m21 * _m.m13 + m22 * _m.m23 + m23 * _m.m33,
            m31 * _m.m11 + m32 * _m.m21 + m33 * _m.m31,
            m31 * _m.m12 + m32 * _m.m22 + m33 * _m.m32,
            m31 * _m.m13 + m32 * _m.m23 + m33 * _m.m33
    );
};


const Matrix3 Matrix3::IDENTITY = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
};
