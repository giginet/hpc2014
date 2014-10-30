//------------------------------------------------------------------------------
/// @file
/// @brief    HPCAction.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCAction.hpp"

namespace hpc {
    //------------------------------------------------------------------------------
    /// Wait用インスタンスを生成します。
    const Action Action::Wait()
    {
        Action action;
        action.mType = ActionType_Wait;
        action.mValue = Vec2(0.0f, 0.0f);
        return action;
    }
    
    //------------------------------------------------------------------------------
    /// Accel用インスタンスを生成します。
    ///
    /// @param[in] aTargetPos 加速する目標座標。
    const Action Action::Accel(const Vec2& aTargetPos)
    {
        Action action;
        action.mType = ActionType_Accel;
        action.mValue = aTargetPos;
        return action;
    }

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Action::Action()
        : mType(ActionType_TERM)
        , mValue()
    {
    }

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Action::Action(ActionType aType, const Vec2& aValue)
        : mType(aType)
        , mValue(aValue)
    {
    }

    //------------------------------------------------------------------------------
    /// 動作を初期化します。
    void Action::reset()
    {
        mType = ActionType_TERM;
        mValue.reset();
    }

    //------------------------------------------------------------------------------
    /// 動作を設定します。
    void Action::set(ActionType aType, const Vec2& aValue)
    {
        mType = aType;
        mValue = aValue;
    }

    //------------------------------------------------------------------------------
    /// 動作の種類を取得します。
    ActionType Action::type()const
    {
        return mType;
    }

    //------------------------------------------------------------------------------
    /// 動作の値を取得します。
    ///
    /// @return 動作の値。
    /// - mType の値が @c ActionType_Wait の場合、無効な値です。
    /// - mType の値が @c ActionType_Accel の場合、加速する目標座標を表します。
    Vec2 Action::value()const
    {
        return mValue;
    }

}

//------------------------------------------------------------------------------
// EOF
