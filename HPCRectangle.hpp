//------------------------------------------------------------------------------
/// @file
/// @brief    Rectangle クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCVec2.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 矩形を表します。
    ///
    /// 矩形は数学直交座標 (左下の値が小さく、右上の値が大きい) で表されます。
    class Rectangle
    {
    public:
        Rectangle();
        Rectangle(float aLeft, float aRight, float aBottom, float aTop);
        Rectangle(const Vec2& aCenter, float aWidth, float aHeight);

        bool isIn(const Vec2& aPos)const;               ///< 点が含まれているかを返します。
        bool isInWithBorder(const Vec2& aPos)const;     ///< 点が含まれているかあるいは境界上にあるかを返します。

        Vec2 center()const;        ///< 中心位置を取得します。
        Vec2 pointA()const;        ///< 点A (左上) の座標を取得します。
        Vec2 pointB()const;        ///< 点B (左下) の座標を取得します。
        Vec2 pointC()const;        ///< 点C (右下) の座標を取得します。
        Vec2 pointD()const;        ///< 点D (右上) の座標を取得します。

        float width()const;        ///< 横幅を取得します。       
        float height()const;       ///< 高さを取得します。

        float left;                 ///< 左を表す x 座標
        float right;                ///< 右を表す x 座標
        float bottom;               ///< 下を表す y 座標
        float top;                  ///< 上を表す y 座標
    };
}
//------------------------------------------------------------------------------
// EOF
