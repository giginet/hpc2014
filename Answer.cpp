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
    int accellPerTurn = 0;
    float wholeDistance = 0;
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
        
        auto lotuses = aStageAccessor.lotuses();
        
        auto player = aStageAccessor.player();
        int maxCount = player.accelCount();
        int waitTurn = player.accelWaitTurn();
        int accellPerTurn = waitTurn / maxCount;
        accellPerTurn += 1; // 切り上げ
        
        int lotusesCount = lotuses.count();
        float lastLength = 0;
        Lotus& prev = lotuses[0];
        for (int i = 0; i < lotusesCount; ++i) {
            Lotus& lotus = lotuses[(i + 1) % lotusesCount];
            float distance = (lotus.pos() - prev.pos()).length();
            prev = lotus;
            wholeDistance += distance;
            if (i == lotusesCount - 1) {
                lastLength = distance;
            }
        }
        // 総距離の算出
        wholeDistance = wholeDistance * 3 - lastLength;
        
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
        const auto ds2 = Parameter::CharaDecelSpeed() * Parameter::CharaDecelSpeed();
        const auto v0 = Parameter::CharaAccelSpeed();
        const auto d = -Parameter::CharaDecelSpeed();
        auto t = -(v0 / d);
        //auto distance =
        // 進む距離

        
        ++sTimer;
        int ac = player.accelCount();
        float speed = player.vel().squareLength();
        // if (ac > 0) {
        if (sTimer >= accellPerTurn) {
            sTimer = 0;
            Vec2 target = lotuses[player.targetLotusNo()].pos();
            auto sub = target - current;
            target -= stream * t;
            return Action::Accel(target);
        }
        return Action::Wait();
    }
}

//------------------------------------------------------------------------------
// EOF
