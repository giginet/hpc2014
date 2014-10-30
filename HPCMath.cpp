//------------------------------------------------------------------------------
/// @file
/// @brief    HPCMath.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCMath.hpp"

#include <cmath>
#include <limits>
#include "HPCCommon.hpp"

namespace hpc {
    //------------------------------------------------------------------------------
    const float Math::PI = 3.14159265359f;

    //------------------------------------------------------------------------------
    /// 2つの値 aLhs, aRhs に対し、許容誤差の範囲内で等しいかどうかを
    /// 判別し返します。
    ///
    /// @param[in] aLhs 値の左辺。
    /// @param[in] aRhs 値の右辺。
    ///
    /// @return aLhs と aRhs が誤差の範囲で等しければ @c true を返し
    ///         そうでない場合は @c false を返します。
    bool Math::IsEqual(float aLhs, float aRhs)
    {
        // 許容誤差を EPS で既定します。
        static const float EPS = 0.0001f;
        const float diff = aLhs - aRhs;

        return -EPS <= diff && diff <= EPS;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aLhs 値の左辺。
    /// @param[in] aRhs 値の右辺。
    ///
    /// @return aLhs, aRhs のうち大きい方を返します。
    float Math::Max(float aLhs, float aRhs)
    {
        return aLhs > aRhs ? aLhs : aRhs;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aLhs 値の左辺。
    /// @param[in] aRhs 値の右辺。
    ///
    /// @return aLhs, aRhs のうち大きい方を返します。
    int Math::Max(int aLhs, int aRhs)
    {
        return aLhs > aRhs ? aLhs : aRhs;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aLhs 値の左辺。
    /// @param[in] aRhs 値の右辺。
    ///
    /// @return aLhs, aRhs のうち小さい方を返します。
    float Math::Min(float aLhs, float aRhs)
    {
        return aLhs < aRhs ? aLhs : aRhs;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aLhs 値の左辺。
    /// @param[in] aRhs 値の右辺。
    ///
    /// @return aLhs, aRhs のうち小さい方を返します。
    int Math::Min(int aLhs, int aRhs)
    {
        return aLhs < aRhs ? aLhs : aRhs;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aValue 入力する値。
    /// @param[in] aMin   値の最小値。
    /// @param[in] aMax   値の最大値。
    ///
    /// @return aValue を [aMin, aMax] の範囲に制限した値を返します。
    float Math::LimitMinMax(float aValue, float aMin, float aMax)
    {
        return Max(Min(aValue, aMax), aMin);
    }

    //------------------------------------------------------------------------------
    /// @param[in] aValue 入力する値。
    /// @param[in] aMin   値の最小値。
    /// @param[in] aMax   値の最大値。
    ///
    /// @return aValue を [aMin, aMax] の範囲に制限した値を返します。
    int Math::LimitMinMax(int aValue, int aMin, int aMax)
    {
        return Max(Min(aValue, aMax), aMin);
    }

    //------------------------------------------------------------------------------
    /// @return aValue の絶対値を返します。
    float Math::Abs(float aValue)
    {
        return aValue >= 0.0f ? aValue : -aValue;
    }

    //------------------------------------------------------------------------------
    /// @return aValue の絶対値を返します。
    int Math::Abs(int aValue)
    {
        return aValue >= 0 ? aValue : -aValue;
    }

    //------------------------------------------------------------------------------
    /// 値の絶対値が aLimitAbs を超えないように aValue を制限します。
    ///
    /// @param[in] aValue    制限対象となる値。
    /// @param[in] aLimitAbs 制限する値。絶対値で与えます。
    ///
    /// @return |aValue| が aLimitAbs を超えないように制限した値を返します。
    float Math::LimitAbs(float aValue, float aLimitAbs)
    {
        HPC_LB_ASSERT_F(aLimitAbs, 0.0f);
        return LimitMinMax(aValue, -aLimitAbs, aLimitAbs);
    }

    //------------------------------------------------------------------------------
    /// @return √aValue の値を返します。
    float Math::Sqrt(float aValue)
    {
        HPC_MIN_ASSERT_F(aValue, 0.0f);
        return std::sqrt(aValue);
    }

    //------------------------------------------------------------------------------
    /// @return aValue を切り上げて整数にした値を返します。
    int Math::Ceil(float aValue)
    {
        return static_cast<int>(std::ceil(aValue));
    }

    //------------------------------------------------------------------------------
    /// @param[in] aValue 判定する値。
    ///
    /// @return aValue に与えられた浮動小数点の値が有効なら @c true を返し、
    ///         無効な値 (inf, nan) の場合は @c false を返します。
    bool Math::IsValid(float aValue)
    {
        return -std::numeric_limits<float>::max() <= aValue &&
            aValue <= std::numeric_limits<float>::max();
    }

    //------------------------------------------------------------------------------
    /// @param[in] aRad 角度をラジアンで指定します。
    ///
    /// @return sin(aRad) の値を返します。
    float Math::Sin(float aRad)
    {
        return std::sin(aRad);
    }

    //------------------------------------------------------------------------------
    /// @param[in] aRad 角度をラジアンで指定します。
    ///
    /// @return cos(aRad) の値を返します。
    float Math::Cos(float aRad)
    {
        return std::cos(aRad);
    }

    //------------------------------------------------------------------------------
    /// @param[in] aRad 角度をラジアンで指定します。
    ///
    /// @return tan(aRad) の値を返します。
    float Math::Tan(float aRad)
    {
        return std::tan(aRad);
    }

    //------------------------------------------------------------------------------
    /// @param[in] aCos 余弦を表す値を指定します。
    ///                 ただし、aCos の値は [-π/2, π/2] にある必要があります。
    ///
    /// @return cos^-1(aCos) の値を返します。
    float Math::ACos(float aCos)
    {
        return std::acos(aCos);
    }

    //------------------------------------------------------------------------------
    /// @return tan^-1(aY / aX) の値を返します。
    float Math::ATan2(float aY, float aX)
    {
        return std::atan2(aY, aX);
    }

    //------------------------------------------------------------------------------
    /// @param[in] aRad 角度をラジアンで指定します。
    ///
    /// @return aRad を度数法(deg) で表した値を返します。
    float Math::RadToDeg(float aRad)
    {
        return aRad * 180.0f / PI;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aDeg 角度を度数で指定します。
    ///
    /// @return aDeg を弧度法(rad) で表した値を返します。
    float Math::DegToRad(float aDeg)
    {
        return aDeg * PI / 180.0f;
    }
}

//------------------------------------------------------------------------------
// EOF
