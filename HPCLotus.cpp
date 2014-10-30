//------------------------------------------------------------------------------
/// @file
/// @brief    HPCLotus.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCLotus.hpp"

#include "HPCCollision.hpp"
#include "HPCCommon.hpp"
#include "HPCParameter.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Lotus::Lotus()
        : mRegion(Vec2(), Parameter::LotusBaseRadius())
    {
    }

    //------------------------------------------------------------------------------
    /// 値を初期化します。
    ///
    /// @param[in] aPos     配置する位置
    /// @param[in] aRadius  半径
    void Lotus::reset(const Vec2& aPos, const float aRadius)
    {
        mRegion.setup(aPos, aRadius);
    }

    //------------------------------------------------------------------------------
    /// 値を無効な蓮となるように初期化します。
    void Lotus::reset()
    {
        mRegion.setPos(Vec2());
    }

    //------------------------------------------------------------------------------
    /// @return 蓮の中心位置を返します。
    Vec2 Lotus::pos()const
    {
        return region().pos();
    }

    //------------------------------------------------------------------------------
    /// @return 蓮の半径を返します。
    float Lotus::radius()const
    {
        return region().radius();
    }

    //------------------------------------------------------------------------------
    /// 蓮が表す領域の円を取得します。
    ///
    /// @return 蓮の領域を表す Circle 型への参照を返します。
    const Circle& Lotus::region()const
    {
        return mRegion;
    }
}

//------------------------------------------------------------------------------
// EOF
