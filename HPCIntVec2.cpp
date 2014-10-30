//------------------------------------------------------------------------------
/// @file
/// @brief    HPCIntVec2.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCIntVec2.hpp"
#include "HPCMath.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 原点またはゼロベクトルを値とするクラスのインスタンスを生成します。
    IntVec2::IntVec2()
        : x(0)
        , y(0)
    {
        reset();
    }

    //------------------------------------------------------------------------------
    /// 初期値を指定してクラスのインスタンスを生成します。
    ///
    /// @param[in] aX  x成分の値
    /// @param[in] aY  y成分の値
    IntVec2::IntVec2(int aX, int aY)
        : x(aX)
        , y(aY)
    {
    }

    //------------------------------------------------------------------------------
    /// 値を (0, 0) に初期化します。
    void IntVec2::reset()
    {
        x = 0;
        y = 0;
    }

    //------------------------------------------------------------------------------
    bool IntVec2::operator==(const IntVec2& aRhs)const
    {
        return x == aRhs.x && y == aRhs.y;
    }

    //------------------------------------------------------------------------------
    IntVec2 IntVec2::operator-()const
    {
        return IntVec2(-x, -y);
    }

    //------------------------------------------------------------------------------
    IntVec2& IntVec2::operator+=(const IntVec2& aRhs)
    {
        x += aRhs.x;
        y += aRhs.y;

        return *this;
    }

    //------------------------------------------------------------------------------
    IntVec2& IntVec2::operator-=(const IntVec2& aRhs)
    {
        return *this += (-aRhs);
    }

    //------------------------------------------------------------------------------
    IntVec2 IntVec2::operator+(const IntVec2& aRhs)const
    {
        IntVec2 v(*this);
        v += aRhs;

        return v;
    }

    //------------------------------------------------------------------------------
    IntVec2 IntVec2::operator-(const IntVec2& aRhs)const
    {
        return *this + (-aRhs);
    }

    //------------------------------------------------------------------------------
    IntVec2& IntVec2::operator*=(int aRhs)
    {
        x *= aRhs;
        y *= aRhs;

        return *this;
    }

    //------------------------------------------------------------------------------
    IntVec2& IntVec2::operator/=(int aRhs)
    {
        x /= aRhs;
        y /= aRhs;

        return *this;
    }
    
    //------------------------------------------------------------------------------
    IntVec2 IntVec2::operator*(int aRhs)const
    {
        IntVec2 v(*this);
        v *= aRhs;
        return v;
    }

    //------------------------------------------------------------------------------
    IntVec2 IntVec2::operator/(int aRhs)const
    {
        IntVec2 v(*this);
        v /= aRhs;
        return v;
    }

    //------------------------------------------------------------------------------
    IntVec2 operator*(int aLhs, const IntVec2& aRhs)
    {
        return aRhs * aLhs;
    }
}

//------------------------------------------------------------------------------
// EOF
