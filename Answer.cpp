//------------------------------------------------------------------------------
/// @file
/// @brief    HPCAnswer.hpp の実装 (解答記述用ファイル)
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

// Answer.cpp 専用のインクルードファイルです。
// 別のファイルをインクルードした場合、評価時には削除されます。
#include "HPCAnswerInclude.hpp"

namespace {
    using namespace hpc;
    
    /// タイマー
    int sTimer = 0;
}

/// プロコン問題環境を表します。
namespace hpc {

    //------------------------------------------------------------------------------
    /// 各ステージ開始時に呼び出されます。
    ///
    /// この関数を実装することで、各ステージに対して初期処理を行うことができます。
    ///
    /// @param[in] aStageAccessor 現在のステージ。
    void Answer::Init(const StageAccessor& aStageAccessor)
    {
        sTimer = 0;
    }

    //------------------------------------------------------------------------------
    /// 各ターンでの動作を返します。
    ///
    /// @param[in] aStageAccessor 現在ステージの情報。
    ///
    /// @return これから行う動作を表す Action クラス。
    Action Answer::GetNextAction(const StageAccessor& aStageAccessor)
    {
        const Chara& player = aStageAccessor.player();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        const Field& field = aStageAccessor.field();
        
        Vec2 stream = field.flowVel();
        Vec2 current = player.pos();
        
        ++sTimer;
        int ac = player.accelCount();
        
        if (ac > 0 ) {
            //sTimer = 0;
            Vec2 target = lotuses[player.targetLotusNo()].pos();
            auto sub = target - current;
            //target += stream * 13;
            return Action::Accel(target);
        }
        return Action::Wait();
    }
}

//------------------------------------------------------------------------------
// EOF
