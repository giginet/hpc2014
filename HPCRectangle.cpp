//------------------------------------------------------------------------------
/// @file
/// @brief    HPCRectangle.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCRectangle.hpp"

#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    /// @note 無効な矩形が生成されます。
    Rectangle::Rectangle()
        : left(0.0f)
        , right(0.0f)
        , bottom(0.0f)
        , top(0.0f)
    {
        left = 0.0f;
        right = 0.0f;
        bottom = 0.0f;
        top = 0.0f;
    }

    //------------------------------------------------------------------------------
    /// 初期値を指定してクラスのインスタンスを生成します。
    ///
    /// @param[in] aLeft    矩形の左側を表す x 座標。
    /// @param[in] aRight   矩形の右側を表す x 座標。
    /// @param[in] aBottom  矩形の下側を表す y 座標。
    /// @param[in] aTop     矩形の上側を表す y 座標。
    Rectangle::Rectangle(float aLeft, float aRight, float aBottom, float aTop)
        : left(aLeft)
        , right(aRight)
        , bottom(aBottom)
        , top(aTop)
    {
        HPC_UB_ASSERT_F(left, right);
        HPC_UB_ASSERT_F(bottom, top);
    }

    //------------------------------------------------------------------------------
    /// 中心位置と縦横の大きさを指定してクラスのインスタンスを生成します。
    ///
    /// @param[in] aCenter  矩形の中心を表す座標。
    /// @param[in] aWidth   矩形の x 方向の長さ。
    /// @param[in] aHeight  矩形の y 方向の長さ。
    Rectangle::Rectangle(const Vec2& aCenter, float aWidth, float aHeight)
        : left(aCenter.x - aWidth / 2.0f)
        , right(aCenter.x + aWidth / 2.0f)
        , bottom(aCenter.y - aHeight / 2.0f)
        , top(aCenter.y + aHeight / 2.0f)
    {
        HPC_UB_ASSERT_F(left, right);
        HPC_UB_ASSERT_F(bottom, top);
    }

    //------------------------------------------------------------------------------
    /// 与えた一点が矩形の内側にあるかどうかを返します。
    /// 点が矩形の辺上にある場合は内側にあるとみなしません。
    ///
    /// @param[in] aPos 調べる点の座標。
    ///
    /// @return aPos が矩形の内部の点ならば @c true を返し、
    ///         そうでなければ @c false を返します。
    bool Rectangle::isIn(const Vec2& aPos)const
    {
        return (left < aPos.x && aPos.x < right) && (bottom < aPos.y && aPos.y < top);
    }

    //------------------------------------------------------------------------------
    /// 与えた一点が矩形の内側にあるかどうかを返します。
    /// 点が矩形の辺上にある場合も内側にあるとみなします。
    ///
    /// @param[in] aPos 調べる点の座標。
    ///
    /// @return aPos が矩形の内部または辺上の点ならば @c true を返し、
    ///         そうでなければ @c false を返します。
    bool Rectangle::isInWithBorder(const Vec2& aPos)const
    {
        return (left <= aPos.x && aPos.x <= right) && (bottom <= aPos.y && aPos.y <= top);
    }

    //------------------------------------------------------------------------------
    float Rectangle::width()const
    {
        return right - left;
    }
    
    //------------------------------------------------------------------------------
    float Rectangle::height()const
    {
        return top - bottom;
    }

    //------------------------------------------------------------------------------
    Vec2 Rectangle::center()const
    {
        return Vec2((right + left) / 2.0f, (top + bottom) / 2.0f);
    }

    //------------------------------------------------------------------------------
    Vec2 Rectangle::pointA()const
    {
        return Vec2(left, top);
    }

    //------------------------------------------------------------------------------
    Vec2 Rectangle::pointB()const
    {
        return Vec2(left, bottom);
    }

    //------------------------------------------------------------------------------
    Vec2 Rectangle::pointC()const
    {
        return Vec2(right, bottom);
    }

    //------------------------------------------------------------------------------
    Vec2 Rectangle::pointD()const
    {
        return Vec2(right, top);
    }
}

//------------------------------------------------------------------------------
// EOF
