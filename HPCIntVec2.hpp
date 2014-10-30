//------------------------------------------------------------------------------
/// @file
/// @brief    IntVec2 クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief 整数値における二次元の値を表します。
    ///
    /// 二次元の要素を表すための型です。
    class IntVec2
    {
    public:
        IntVec2();
        IntVec2(int x, int y);

        void reset();                               ///< 値をゼロ値で初期化する。

        bool operator==(const IntVec2& aRhs)const;
        IntVec2 operator-()const;
        IntVec2& operator+=(const IntVec2& aRhs);
        IntVec2& operator-=(const IntVec2& aRhs);
        IntVec2& operator*=(int aRhs);
        IntVec2& operator/=(int aRhs);
        IntVec2 operator+(const IntVec2& aRhs)const;
        IntVec2 operator-(const IntVec2& aRhs)const;
        IntVec2 operator*(int aRhs)const;
        IntVec2 operator/(int aRhs)const;
        friend IntVec2 operator*(int aLhs, const IntVec2& aRhs);

        int x;  ///< x 要素
        int y;  ///< y 要素
    };
}
//------------------------------------------------------------------------------
// EOF
