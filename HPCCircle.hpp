//------------------------------------------------------------------------------
/// @file
/// @brief    Circle クラス
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
    /// 円を表します。
    class Circle
    {
    public:
        Circle(const Vec2& aPos, float aRadius);

        void setup(const Vec2& aPos, float aRadius);    ///< 値を設定します。
        void setPos(const Vec2& aPos);                  ///< 中心座標を設定します。
        void setRadius(float aRadius);                  ///< 半径を設定します。
        void move(const Vec2& aVec);                    ///< 平行に移動します。

        const Vec2& pos()const;                        ///< 中心座標を取得します。
        float radius()const;                           ///< 半径を取得します。

    private:
        Vec2 mPos;          ///< 中心座標
        float mRadius;      ///< 半径
    };
}
//------------------------------------------------------------------------------
// EOF
