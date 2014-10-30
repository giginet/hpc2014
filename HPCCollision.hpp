//------------------------------------------------------------------------------
/// @file
/// @brief    Collision クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCCircle.hpp"
#include "HPCRectangle.hpp"
#include "HPCVec2.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 衝突判定用のユーティリティ関数を提供します。
    class Collision
    {
    public:
        /// 静止している2つの円が衝突するかどうかを返します。
        static bool IsHit(const Circle& aC0, const Circle& aC1);
        /// 静止している円と、移動している円が衝突するかどうかを返します。
        static bool IsHit(const Circle& aC0, const Circle& aC1, const Vec2& aP1);

    private:
        Collision();
    };
}
//------------------------------------------------------------------------------
// EOF
