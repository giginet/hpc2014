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
#include <iostream>

namespace {
    using namespace hpc;
    
    /// タイマー
    int sTimer = 0;
    float accelPerTurn = 0;
    float wholeDistance = 0;
    int stageNo = 0;
    int lastTargetLotusNo = 0;
    
    // 最後にアクセルを踏んだ地点
    Vec2 lastAccellPos;
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
        sTimer = 1000;
        accelPerTurn = 0;
        wholeDistance = 0;
        
        auto lotuses = aStageAccessor.lotuses();
        const Chara& player = aStageAccessor.player();
        
        Vec2 initialPosition = player.pos();
        float initialDistance = (lotuses[0].pos() - initialPosition).length();
        
        int maxCount = player.accelCount();
        int waitTurn = player.accelWaitTurn();
        
        int lotusesCount = lotuses.count();
        float lastDistance = 0;
        Lotus& prev = lotuses[0];
        for (int i = 0; i < lotusesCount; ++i) {
            Lotus& lotus = lotuses[(i + 1) % lotusesCount];
            float distance = (lotus.pos() - prev.pos()).length();
            prev = lotus;
            wholeDistance += distance;
            if (i == lotusesCount - 1) {
                lastDistance = distance;
            }
        }
        // 総距離の算出
        wholeDistance = wholeDistance * 3 - lastDistance + initialDistance;
        
        float v0 = Parameter::CharaInitAccelCount;
        float a = -1 * Parameter::CharaAccelSpeed();
        float stopTime = v0 / -a;
        
        // 1回のアクセルで進める総距離
        const float distancePerAccell = (-(v0* v0) / (2 * a));
        
        // 突破までに必要な最低アクセル回数
        auto requiredAccelCount = wholeDistance / distancePerAccell;
        // 最低限アクセルを使ったときのクリアターン数
        const float allTime = requiredAccelCount * stopTime;
        
        accelPerTurn = 0;
        if (waitTurn > 0) {
            accelPerTurn = allTime / ((allTime / waitTurn) + player.accelCount());
        } else {
            accelPerTurn = allTime / player.accelCount();
        }
        accelPerTurn += 0.08; // 切り上げ
        //std::cout << "Stage : " << stageNo << std::endl;
        //std::cout << accelPerTurn << std::endl;
        lastTargetLotusNo = player.targetLotusNo();
        ++stageNo;
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

        
        bool doAccel = false;
        int ac = player.accelCount();
        // if (ac > 0) {
        
        // 前回と変わってたら
        if (lastTargetLotusNo != player.targetLotusNo()) {
            doAccel = true;
        }
        
        if (sTimer >= accelPerTurn) {
            doAccel = true;
        }
        
        lastTargetLotusNo = player.targetLotusNo();
        
        if (doAccel) {
            sTimer = 0;
            Vec2 target = lotuses[player.targetLotusNo()].pos();
            Vec2 nextTarget = lotuses[(player.targetLotusNo() + 1) % lotuses.count()].pos();
            auto sub = nextTarget - target;
            auto normalized = sub;
            normalized.normalize();
            
            Vec2 goal = target + normalized * lotuses[player.targetLotusNo()].radius();
            
            goal -= stream * t;
            if ( !isReachInCurrentAccel(player, target) ) {
                return Action::Accel(goal);
            }
        }
        
        ++sTimer;
        return Action::Wait();
    }
    
    // 今のアクセルで到達可能な地点
    Vec2 canReachCurrentAccelPos(const Chara& player) {
        auto v0 = player.vel().length();
        auto a = Parameter::CharaDecelSpeed();
        auto length = (-(v0* v0) / (2 * a));
        auto normalized = player.vel();
        normalized.normalize();
        Vec2 currentPos = player.pos();
        return currentPos + normalized * length;
    }
    
    // targetに現在のアクセルだけで到達可能かどうか
    bool Answer::isReachInCurrentAccel(const Chara& player, Vec2 target)
    {
        if ( (player.pos() - target).squareLength() < Parameter::CharaRadius() * 2 ) {
            return true;
        }
        Vec2 goal = canReachCurrentAccelPos(player);
        return (goal - target).squareLength() <= Parameter::CharaRadius() * 2;
    }
    
    
}

//------------------------------------------------------------------------------
// EOF
