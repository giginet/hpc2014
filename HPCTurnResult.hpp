//------------------------------------------------------------------------------
/// @file
/// @brief    TurnResult クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCParameter.hpp"
#include "HPCStageState.hpp"
#include "HPCVec2.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 1ターンの実行に対する結果を表します。
    struct TurnResult 
    {
        TurnResult();

        void reset();                                                   ///< 情報を初期化します。
        void set(const TurnResult& aResult);                            ///< 値を上書き設定します。

        /// キャラ情報
        struct Chara
        {
            Vec2 pos;
            int accelCount;
            int passedLotusCount;
        } charas[Parameter::CharaCountMax];
        
        StageState state;   ///< 現在の状態
    };
}
//------------------------------------------------------------------------------
// EOF
