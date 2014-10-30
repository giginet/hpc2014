//------------------------------------------------------------------------------
/// @file
/// @brief    HPCField.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCField.hpp"

#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCParameter.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Field::Field()
        : mRect()
        , mFlowVel()
    {
    }

    //------------------------------------------------------------------------------
    /// 初期設定を行います。
    void Field::setup(
        const Rectangle& aRect
        , const Vec2& aFlowVel
        )
    {
        mRect = aRect;
        mFlowVel = aFlowVel;
    }

    //------------------------------------------------------------------------------
    /// フィールド情報を設定します。
    void Field::set(const Field& aField)
    {
        mRect = aField.rect();
        mFlowVel = aField.flowVel();
    }

    //------------------------------------------------------------------------------
    /// 初期化します。
    void Field::reset()
    {
        mRect = Rectangle();
        mFlowVel.reset();
    }

    //------------------------------------------------------------------------------
    /// @return 矩形。
    const Rectangle& Field::rect()const
    {
        return mRect;
    }

    //------------------------------------------------------------------------------
    /// @return 流れる速度。
    const Vec2& Field::flowVel()const
    {
        return mFlowVel;
    }
}

//------------------------------------------------------------------------------
// EOF
