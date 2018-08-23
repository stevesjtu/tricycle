#ifndef PREDEF_H
#define PREDEF_H

#include"Eigen/Dense"
#include<iostream>

#define TRICYCLE_DLL

#define TRICYCLE_EXPORTS

namespace Eigen {

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//																										 //
// add unsigned type for Dense and Array																 //
//																										 //
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

#define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size)         \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, Dynamic> Matrix##Size##X##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Dynamic, Size> Matrix##X##Size##TypeSuffix;

#define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Dynamic, X) \
EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 2) \
EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 3) \
EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 4)

EIGEN_MAKE_TYPEDEFS_ALL_SIZES(unsigned, u)

#undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
#undef EIGEN_MAKE_TYPEDEFS
#undef EIGEN_MAKE_FIXED_TYPEDEFS

    ///////////////////////////////////////////////////////////////////////////////////////////////////
#define EIGEN_MAKE_ARRAY_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
/** \ingroup arraytypedefs */                                    \
typedef Array<Type, Size, Size> Array##SizeSuffix##SizeSuffix##TypeSuffix;  \
/** \ingroup arraytypedefs */                                    \
typedef Array<Type, Size, 1>    Array##SizeSuffix##TypeSuffix;

#define EIGEN_MAKE_ARRAY_FIXED_TYPEDEFS(Type, TypeSuffix, Size)         \
/** \ingroup arraytypedefs */                                    \
typedef Array<Type, Size, Dynamic> Array##Size##X##TypeSuffix;  \
/** \ingroup arraytypedefs */                                    \
typedef Array<Type, Dynamic, Size> Array##X##Size##TypeSuffix;

#define EIGEN_MAKE_ARRAY_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
EIGEN_MAKE_ARRAY_TYPEDEFS(Type, TypeSuffix, 2, 2) \
EIGEN_MAKE_ARRAY_TYPEDEFS(Type, TypeSuffix, 3, 3) \
EIGEN_MAKE_ARRAY_TYPEDEFS(Type, TypeSuffix, 4, 4) \
EIGEN_MAKE_ARRAY_TYPEDEFS(Type, TypeSuffix, Dynamic, X) \
EIGEN_MAKE_ARRAY_FIXED_TYPEDEFS(Type, TypeSuffix, 2) \
EIGEN_MAKE_ARRAY_FIXED_TYPEDEFS(Type, TypeSuffix, 3) \
EIGEN_MAKE_ARRAY_FIXED_TYPEDEFS(Type, TypeSuffix, 4)

EIGEN_MAKE_ARRAY_TYPEDEFS_ALL_SIZES(unsigned, u)

#undef EIGEN_MAKE_ARRAY_TYPEDEFS_ALL_SIZES
#undef EIGEN_MAKE_ARRAY_TYPEDEFS
#undef EIGEN_MAKE_ARRAY_TYPEDEFS_LARGE

#undef EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE
#undef EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE
#undef EIGEN_USING_ARRAY_TYPEDEFS

#define EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE(TypeSuffix, SizeSuffix) \
using Eigen::Matrix##SizeSuffix##TypeSuffix; \
using Eigen::Vector##SizeSuffix##TypeSuffix; \
using Eigen::RowVector##SizeSuffix##TypeSuffix;

#define EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE(TypeSuffix) \
EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE(TypeSuffix, 2) \
EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE(TypeSuffix, 3) \
EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE(TypeSuffix, 4) \
EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE(TypeSuffix, X) \

#define EIGEN_USING_ARRAY_TYPEDEFS \
EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE(u)

#undef EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE_AND_SIZE
#undef EIGEN_USING_ARRAY_TYPEDEFS_FOR_TYPE
#undef EIGEN_USING_ARRAY_TYPEDEFS
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//																										 //
// typedef some types, and const matrices																 //
//																										 //
///////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef MatrixXd MatX;
typedef Matrix3d Mat3;

typedef VectorXd VecX;
typedef Vector3d Vec3;

typedef Ref<MatrixXd> RefmX;
typedef const Ref<const MatrixXd>& RefCmX;

typedef Ref<Matrix3d> Refm3;
typedef const Ref<const Matrix3d>& RefCm3;

typedef Ref<VectorXd> RefvX;
typedef const Ref<const VectorXd>& RefCvX;

typedef Ref<Vector3d> Refv3;
typedef const Ref<const Vector3d>& RefCv3;

typedef Map<Vector3d> Mapv3;
typedef Map<const Vector3d> MapCv3;

typedef Map<VectorXd> MapvX;
typedef Map<const VectorXd> MapCvX;

const Matrix3d I3 = Matrix3d::Identity();
const Matrix3d Z3 = Matrix3d::Zero();
const Vector3d ZV3 = Vector3d::Zero();
const Vector3d E1 = Vector3d(1.0, 0.0, 0.0);
const Vector3d E2 = Vector3d(0.0, 1.0, 0.0);
const Vector3d E3 = Vector3d(0.0, 0.0, 1.0);

}

#include<memory>
#include<vector>
#include <iomanip>

#if((defined _WIN32) && (defined TRICYCLE_DLL))
  #ifdef TRICYCLE_EXPORTS
    #define TRICYCLE_API __declspec(dllexport)
  #else
    #define TRICYCLE_API __declspec(dllimport)
  #endif
#else
  #define TRICYCLE_API
#endif

// disable warning when exporting template to dll
#pragma warning(disable : 4251)

#ifdef _MSC_VER
#define _SCL_SECURE_NO_WARNINGS
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif

#define New std::make_shared

namespace tricycle {

template<class T>
using sptr = std::shared_ptr<T>;

template<class T>
using uptr = std::unique_ptr<T>;

template<class T>
using Collection = std::vector<sptr<T> >;

template<typename T>
inline bool TRICYCLE_API equals(const T &c1, const T &c2, const T &tor) {
  return abs(double(c1 - c2)) < tor ? true : false;
}

template<typename T>
inline void solve22(const Eigen::Matrix2d &A, const T &b, T &xi) {
  Eigen::Matrix2d B;
  B(0, 0) = A(1, 1);
  B(0, 1) = -A(0, 1);
  B(1, 0) = -A(1, 0);
  B(1, 1) = A(0, 0);
  xi = 1.0 / (A(0, 0)*A(1, 1) - A(0, 1)* A(1, 0))*B*b;
}


}


#endif
