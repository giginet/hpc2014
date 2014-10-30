//------------------------------------------------------------------------------
/// @file
/// @brief    HPCCollision.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCCollision.hpp"
#include "HPCMath.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 静止している2つの円が交差しているかどうかを返します。
    /// 接している場合も交差とみなします。
    ///
    /// @param[in] aC0    1つ目の円
    /// @param[in] aC1    2つ目の円
    ///
    /// @return 2つの円が衝突していたら @c true 。そうでなければ @c false 。
    ///         接しているときも衝突とみなします。
    bool Collision::IsHit(const Circle& aC0, const Circle& aC1)
    {
        const float squareDist = aC0.pos().squareDist(aC1.pos());
        return squareDist <= (aC0.radius() + aC1.radius()) * (aC0.radius() + aC1.radius());
    }

    //------------------------------------------------------------------------------
    /// 静止している円と、動いている円との衝突を判定します。
    /// 接している場合も交差とみなします。
    ///
    /// @param[in] aC0    静止している円。
    /// @param[in] aC1    動いている円。移動開始時の状態を設定します。
    /// @param[in] aP1    動いている円の移動後の位置。
    ///
    /// @return 2つの円が衝突していたら @c true 。そうでなければ @c false 。
    ///         接しているときも衝突とみなします。
    bool Collision::IsHit(const Circle& aC0, const Circle& aC1, const Vec2& aP1)
    {
        // 動いていなければ静止している円の判定
        if (aC1.pos() == aP1) {
            return IsHit(aC0, aC1);
        }

        // 実質的に半径 aC0.radius + aC1.radius の円と線分 [aC1.pos, aP1.pos] の衝突判定と
        // 同等になる。
        const Circle c(aC0.pos(), aC0.radius() + aC1.radius());
        const Vec2 seg = aP1 - aC1.pos();
        const Vec2 c1ToC0 = c.pos() - aC1.pos();
        const float dist = Math::Abs(seg.cross(c1ToC0)) / seg.length();
        // 距離が c.radius より遠ければ衝突しない
        if (dist > c.radius()) {
            return false;
        }
        // 線分の延長線上で交差していないか調べる。
        const Vec2 p1ToC0 = c.pos() - aP1;
        // それぞれの点が円の反対方向にあれば衝突
        if (c1ToC0.dot(seg) * p1ToC0.dot(seg) <= 0.0f) {
            return true;
        }
        // 半径が大きければ衝突
        if (c.radius() >= c1ToC0.length() || c.radius() >= p1ToC0.length()) {
            return true;
        }
        return false;
    }
}

//------------------------------------------------------------------------------
// EOF
