//------------------------------------------------------------------------------
/// @file
/// @brief      Action クラス
/// @author     ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCActionType.hpp"
#include "HPCVec2.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 1ターン毎の動作の内容を表します。
    ///
    /// Answer::GetNextAction から返される、次の動作を決定づけるデータを表します。
    /// 
    /// @attention ActionType_Accelを使用した際、以下のいずれかの条件を満たすと
    ///            ActionType_Waitと同等に扱われます。
    ///            ・キャラの accelCount() がゼロの時。
    ///            ・キャラの pos() と mValue が同値の時。
    class Action
    {
    public:
        /// Wait用インスタンスを返します。
        static const Action Wait();
        /// Accel用インスタンスを返します。
        static const Action Accel(const Vec2& aTargetPos);
        
    public:
        Action();
        Action(ActionType aType, const Vec2& aValue);

        void reset();                   ///< 動作を初期化します。
        void set(ActionType aType, const Vec2& aValue); ///< 動作を設定します。
        
        ActionType type()const;         ///< 動作の種類を取得します。
        Vec2 value()const;              ///< 動作の値を取得します。

    private:
        ActionType mType;               ///< 動作の種類
        Vec2 mValue;                    ///< 動作の値
    };
}
//------------------------------------------------------------------------------
// EOF
