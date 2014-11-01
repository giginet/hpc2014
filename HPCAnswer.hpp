//------------------------------------------------------------------------------
/// @file
/// @brief    Answer クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCAction.hpp"
#include "HPCChara.hpp"
#include "HPCStageAccessor.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ゲームの解答を表します。
    ///
    /// 参加者は Answer.cpp にこのクラスのメンバ関数 Init, GetNextAction 
    /// を実装することで、プログラムを作成します。
    class Answer
    {
    public:
        static void Init(const StageAccessor& aStageAccessor);              ///< 各ステージ開始時に呼び出されます。
        static Action GetNextAction(const StageAccessor& aStageAccessor);   ///< 次の動作を決定します。
        static bool isReachInCurrentAccel(const Chara& player, Vec2 target);
    private:
        Answer();
    };
}
//------------------------------------------------------------------------------
// EOF
