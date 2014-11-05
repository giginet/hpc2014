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
    
    int lastTargetLotusNo = 0;
    bool changedTarget = false;
    Vec2 lastPlayerPosition;
    Vec2 initialPlayerPosition;
    
    float wholeDistance = 0;
    float accelPerTurn = 0;
    
    // 最後にアクセルを踏んだ地点
    Vec2 lastAccellPos;
    
    // デバッグ用
    int stageNo = 0;
    float realDistance = 0;
    
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
        ac.rotate(Math::DegToRad(90));
        
        ac.normalize();
        
        ac *= (target.radius() * 0.5);
        
        // 逆バージョンも作る
        Vec2 reversed = ac * -1;
        
        Vec2 target0 = target.pos() + ac;
        Vec2 target1 = target.pos() + reversed;
        
        // 近い方を返す
        float distance0 = (target0 - nextPoint).squareLength() + (target0 - prevPoint).squareLength();
        float distance1 = (target1 - nextPoint).squareLength() + (target1 - prevPoint).squareLength();
        if (distance0 < distance1) {
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
            Vec2 prevPoint = initialPlayerPosition; // 初期位置
            Vec2 nextPoint = lotuses[1].pos(); // 1個目
            const Lotus& target = lotuses[0];
            goal = getTargetByThreePoints(target, prevPoint, nextPoint);
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
        for (int i = 0; i < lotusesCount; ++i) {
            const Lotus& lotus = lotuses[i];
            const Lotus& next = lotuses[(i + 1) % lotuses.count()];
            distance += (next.pos() - lotus.pos()).length();
        }
        // 総距離の算出
        distance = distance * 3 + initialDistance;
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
        //std::cout << "rd" << realDistance << std::endl;
        realDistance = 0;
        changedTarget = false;
        
        wholeDistance = 0;
        accelPerTurn = 0;
        
        const Chara& player = aStageAccessor.player();
        initialPlayerPosition = player.pos();
        lastPlayerPosition = player.pos();
        
        
        float v0 = Parameter::CharaInitAccelCount;
        float a = -1 * Parameter::CharaAccelSpeed();
        float stopTime = v0 / -a;
        float waitTurn = player.accelWaitTurn();
        
        // 平均速度を算出する
        float averageSpeed = 0;
        float wd = calcWholeDistance(aStageAccessor);
        for (float as = 3.0; as < v0; as += 0.01 ) {
            // 予想ターン数
            // 仮でざっくり実装する
            float estimateTurn = wd / as;
            int accelCount = (estimateTurn / waitTurn) + player.accelCount();
            
        }
        
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
        const EnemyAccessor& enemies = aStageAccessor.enemies();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        //auto distance =
        // 進む距離
        
        int maxLotusCount = lotuses.count();
        
        // 最低限残しておくアクセル回数
        int saveAccelThreshold = 2;
        /*if (player.passedLotusCount() > maxLotusCount / 2) {
            // 最後の方は自重しなくする
            saveAccelThreshold = 2;
        }*/
        
        bool doAccel = false;
        // if (ac > 0) {
        
        int loopCount = (int)(player.passedLotusCount() / aStageAccessor.lotuses().count()) + 1;
        Vec2 goal = getNextTarget(player.targetLotusNo(), aStageAccessor, loopCount);
        Vec2 sub = goal - player.pos();
        
        // 前回と目的地が変わってたらアクセル踏み直す
        if (lastTargetLotusNo != player.targetLotusNo() && player.accelCount() >= saveAccelThreshold - 1) {
            doAccel = true;
            changedTarget = true;
            sTimer = -5;
        } else {
            // 前回と目的地は変わってないけど、前回より遠くなってたら即座に方向転換する
            float currentDistance = (goal - player.pos()).squareLength();
            float prevDistance = (goal - lastPlayerPosition).squareLength();
            if (currentDistance > prevDistance) {
                doAccel = true;
                //sTimer = -5;
            }
        }
        
        // lastPlayerPosition更新
        lastPlayerPosition = player.pos();
        
        if (sTimer >= player.accelWaitTurn() && player.accelCount() >= saveAccelThreshold) {
            // これ以上加速しなくてもたどり着けそうなら加速しない
            float stopTurn = player.vel().length() / Parameter::CharaDecelSpeed();
            float v0 = player.vel().length();
            float a = -Parameter::CharaDecelSpeed();
            float length = v0 * stopTurn + 0.5 * a * stopTurn * stopTurn;
            if (sub.length() >= length) {
                doAccel = true;
            }
        }
        
        if (changedTarget) {
            // ターゲットが変わってたらdoaccel
            doAccel = true;
        }
        
        lastTargetLotusNo = player.targetLotusNo();
        if (doAccel) {
            if (player.accelCount() > 0) {
                if (sTimer > 0) sTimer = 0;
                changedTarget = false;
                //if ( !isReachInCurrentAccel(player, goal) ) {
                return Action::Accel(goal);
                //}
            }
        } else {
            ++sTimer;
        }
        realDistance += (player.pos() - lastPlayerPosition).length();
        
        return Action::Wait();
    }
    
}

//------------------------------------------------------------------------------
// EOF
