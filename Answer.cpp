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

    // 加速してからtターン後の位置を返します
    float distanceAfterTurn(float t, const StageAccessor& aStageAccessor)
    {
        const Chara& player = aStageAccessor.player();
        float v0 = Parameter::CharaAccelSpeed();
        float a = -Parameter::CharaDecelSpeed();
        if (t >= v0 / -a) {
            t = v0 / -a;
        }
        return v0 * t + 0.5 * a * t * t;
    }
    
    /// ハスa, b, cを通るときに、一番いい感じでbに接触できるようなb点を返します
    Vec2 getTargetByThreePoints(const Lotus& target, Vec2 prevPoint, Vec2 nextPoint)
    {
        // acベクトルを生成
        Vec2 ac = nextPoint - prevPoint;
        
        // acベクトルを90度回転する
        ac.rotate(90);
        
        ac.normalize();
        
        ac *= target.radius();
        
        // 逆バージョンも作る
        Vec2 reversed = ac;
        reversed *= -1;
        
        Vec2 target0 = target.pos() + ac;
        Vec2 target1 = target.pos() + reversed;
        
        // 近い方を返す
        if ((target0 - nextPoint).squareLength() < (target1 - nextPoint).squareLength() ) {
            return target0;
        }
        return target1;
    }
    
    Vec2 getNextTarget(int targetLotusNo, const StageAccessor& aStageAccessor, int loopNo)
    {
        
        const float v0 = Parameter::CharaAccelSpeed();
        const float d = -Parameter::CharaDecelSpeed();
        float t = -(v0 / d);
        
        const Chara& player = aStageAccessor.player();
        const Field& field = aStageAccessor.field();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        int lotusCount = lotuses.count();
        
        Vec2 goal;
        if (targetLotusNo == 0 && loopNo == 1) {
            // まだ一つも回ってないとき
            // 初期位置、1個目、2個目で三角形を作る
            Vec2 prevPoint = player.pos(); // 初期位置
            Vec2 nextPoint = lotuses[1].pos(); // 1個目
            const Lotus& target = lotuses[0];
            goal = getTargetByThreePoints(target, prevPoint, nextPoint);
        } else if (targetLotusNo == lotusCount - 1 && loopNo == 3) {
            // 最後の一つのとき
            // 2点だけを参照する
            Vec2 target = lotuses[targetLotusNo].pos();
            Vec2 prevTarget = lotuses[(targetLotusNo - 1 + lotusCount) % lotuses.count()].pos();
            Vec2 sub = target - prevTarget;
            Vec2 normalized = sub;
            normalized.normalize();
            
            goal = target + normalized * lotuses[targetLotusNo].radius();
        } else {
            // それ以外の時
            Vec2 prevPoint = lotuses[(targetLotusNo - 1 + lotusCount) % lotusCount].pos(); // 1つ前
            Vec2 nextPoint = lotuses[(targetLotusNo + 1) % lotusCount].pos(); // 1つ後
            const Lotus& target = lotuses[targetLotusNo];
            goal = getTargetByThreePoints(target, prevPoint, nextPoint);
        }
        Vec2 stream = field.flowVel();
        
        goal -= stream * t;
        return goal;
    }
    
    // 今のアクセルで到達可能な地点
    Vec2 canReachCurrentAccelPos(const Chara& player) {
        float v0 = player.vel().length();
        float a = Parameter::CharaDecelSpeed();
        float length = (-(v0* v0) / (2 * a));
        Vec2 normalized = player.vel();
        normalized.normalize();
        Vec2 currentPos = player.pos();
        return currentPos + normalized * length;
    }
    
    // targetに現在のアクセルだけで到達可能かどうか
    bool isReachInCurrentAccel(const Chara& player, Vec2 target)
    {
        if ( (player.pos() - target).squareLength() < Parameter::CharaRadius() * 2 ) {
            return true;
        }
        Vec2 goal = canReachCurrentAccelPos(player);
        return (goal - target).squareLength() <= Parameter::CharaRadius() * 2;
    }
    
    float calcWholeDistance(const StageAccessor& aStageAccessor)
    {
        float distance = 0;
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        const Chara& player = aStageAccessor.player();
        
        int lotusesCount = lotuses.count();
        Vec2 current = player.pos();
        Vec2 firstTarget = getNextTarget(0, aStageAccessor, 1);
        // 最初の距離
        float initialDistance = (firstTarget - current).length();
        current = firstTarget;
        float lastDistance = 0;
        for (int i = 0; i < lotusesCount * 3; ++i) {
            int index = i % 3;
            int loopCount = Math::Ceil(i / 3);
            int nextLoopCount = Math::Ceil((i + 1) / 3);
            Vec2 target = getNextTarget(index, aStageAccessor, loopCount);
            Vec2 nextTarget = getNextTarget((index + 1) % lotuses.count(), aStageAccessor, nextLoopCount);
            
            float partialDistance = (nextTarget - target).length();
            distance += partialDistance;
            
        }
        // 総距離の算出
        distance += initialDistance;
        return distance;
    }
    
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
        
        
        float v0 = Parameter::CharaInitAccelCount;
        float a = -1 * Parameter::CharaAccelSpeed();
        float stopTime = v0 / -a;
        const Chara& player = aStageAccessor.player();
        
        int waitTurn = player.accelWaitTurn();
        
        wholeDistance = calcWholeDistance(aStageAccessor);
        
        // 1回のアクセルで進める総距離
        const float distancePerAccell = (-(v0* v0) / (2 * a));
        
        /*
        // 突破までに必要な最低アクセル回数
        float requiredAccelCount = wholeDistance / distancePerAccell;
        // 最低限アクセルを使ったときのクリアターン数
        // estimateの初期値
        const float estimateTurn = requiredAccelCount * stopTime * 8.8;
        
        */
        
        float estimateTurn = 0;
        // accelPerTurnを推測する
        for (int apt = 1; apt < 100; ++apt) {
            float distancePerAccel = distanceAfterTurn(apt, aStageAccessor);
            float et = wholeDistance * 8.0 / distancePerAccel;
            float requiredAccelCount = et / apt;
            float allEnableAccelCount = player.accelCount() + et / player.accelWaitTurn();
            if (allEnableAccelCount >= requiredAccelCount) {
                estimateTurn = et;
                accelPerTurn = apt;
                break;
            }
        }
        
        //accelPerTurn = 0;
        //accelPerTurn = estimateTurn / ((estimateTurn / waitTurn) + player.accelCount());
        //std::cout << "Stage : " << stageNo << std::endl;
        //std::cout << wholeDistance << std::endl;
        //std::cout << distancePerAccell << std::endl;
        //std::cout << "et " << estimateTurn << std::endl;
        //std::cout << "apt " << accelPerTurn << std::endl;
        //std::cout << "wd " << wholeDistance << std::endl;

        
        
        // 最後に通ったハス
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
        //auto distance =
        // 進む距離
        
        
        bool doAccel = false;
        // if (ac > 0) {
        
        // 前回と目的地が変わってたらアクセル踏み直す
        if (lastTargetLotusNo != player.targetLotusNo()) {
            doAccel = true;
        }
        
        if (sTimer >= accelPerTurn) {
            doAccel = true;
        }
        
        lastTargetLotusNo = player.targetLotusNo();
        int loopCount = Math::Ceil(player.passedLotusCount() / aStageAccessor.lotuses().count());
        
        if (doAccel) {
            sTimer = 0;
            Vec2 goal = getNextTarget(player.targetLotusNo(), aStageAccessor, loopCount);
            if ( !isReachInCurrentAccel(player, goal) ) {
                return Action::Accel(goal);
            }
        }
        
        ++sTimer;
        return Action::Wait();
    }
    
}

//------------------------------------------------------------------------------
// EOF
