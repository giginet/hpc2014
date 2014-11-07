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
    
    int lastTargetLotusNo = 0;
    bool changedTarget = false;
    Vec2 initialPlayerPosition;
    
    float wholeDistance = 0;
    float minSpeed = 0;
    
    // 最後にアクセルを踏んだ地点
    Vec2 lastAccellPos;
    
    // 過去の移動履歴
    Vec2 positionHistory[Parameter::GameTurnPerStage];
}

/// プロコン問題環境を表します。
namespace hpc {
    
    /// ハスa, b, cを通るときに、一番いい感じでbに接触できるような点を返します
    Vec2 getTargetByThreePoints(const Lotus& target, Vec2 prevPoint, Vec2 nextPoint)
    {
        // acベクトルを生成
        Vec2 ac = nextPoint - prevPoint;
        
        // acベクトルを90度回転する
        ac.rotate(Math::DegToRad(90));
        
        ac.normalize();
        
        ac *= target.radius() * 0.8;
        
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
    
    // ハスa, bを通るときに、一番いい感じでaに接触できるような点を返します
    Vec2 getTargetByTwoPoints(const Lotus& target, Vec2 nextPoint)
    {
        // abベクトルを生成
        Vec2 ab = nextPoint - target.pos();
        ab.normalize();
        
        Vec2 goal = target.pos() + ab * target.radius();
        return goal;
    }
    
    // 次の目的地を返します
    Vec2 getNextTarget(const StageAccessor& aStageAccessor)
    {
        const float v0 = Parameter::CharaAccelSpeed();
        const float d = -Parameter::CharaDecelSpeed();
        float t = -(v0 / d);
        const Chara& player = aStageAccessor.player();
        const Field& field = aStageAccessor.field();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        int lotusCount = lotuses.count();
        
        int targetLotusNo = player.targetLotusNo();
        int roundNo = player.roundCount();
        // もし、targetが最後ハスだったら
        if (roundNo == 2 && targetLotusNo == lotusCount - 1)
        {
            const Lotus& lotus = lotuses[targetLotusNo];
            Vec2 sub = player.pos() - lotus.pos();
            sub.normalize();
            return lotus.pos() + sub * (lotus.radius() + Parameter::CharaRadius());
        } else {
            Vec2 goal;
            
            // それ以外の時
            // 1つ先のゴールを出す
            Vec2 prevPoint;
            if (roundNo == 0 && targetLotusNo == 0) {
                // まだ1つも通ってないとき、最初の点を使う
                prevPoint = initialPlayerPosition;
            } else {
                // それ以外の時、1個前の点を使う
                prevPoint = lotuses[(targetLotusNo - 1 + lotusCount) % lotusCount].pos();
            }
            const Lotus& target = lotuses[targetLotusNo];
            const Lotus& target2 = lotuses[(targetLotusNo + 1) % lotusCount];
            Vec2 goalA0 = getTargetByThreePoints(target, prevPoint, target2.pos());
            Vec2 goalB0 = getTargetByTwoPoints(target, target2.pos());
            
            // 2つ先のゴールを出す
            Vec2 nextPoint2 =lotuses[(targetLotusNo + 2) % lotusCount].pos(); // 2つあと
            Vec2 goalA1 = getTargetByThreePoints(target2, target.pos(), nextPoint2);
            Vec2 goalB1 = getTargetByTwoPoints(target2, nextPoint2);
            if ((goalA0 + goalA1).squareLength() < (goalB0 + goalB1).squareLength()) {
                // Aルートの方が近い
                goal = goalA0;
            } else {
                // Bルートの方が近い
                goal = goalB0;
            }
            
            
            Vec2 stream = field.flowVel();
            goal -= stream * t;
            return goal;
        }
        
    }
    
    // nターン後の位置を返す
    Vec2 posAfterTurn(const StageAccessor& stageAccessor, int afterTurn) {
        const Chara& player = stageAccessor.player();
        const Field& field = stageAccessor.field();
        float stopTurn = player.vel().length() / Parameter::CharaDecelSpeed();
        if (afterTurn > stopTurn)
        {
            afterTurn = stopTurn;
        }
        float a = -Parameter::CharaDecelSpeed();
        Vec2 currentVel = player.vel();
        Vec2 currentPos = player.pos();
        for (int passedTurn = 1; passedTurn <= afterTurn; ++passedTurn) {
            currentPos = currentPos + currentVel + field.flowVel();
            float currentSpeed = player.vel().length();
            currentVel.normalize();
            currentSpeed += a;
            currentVel *= currentSpeed;
        }
        return currentPos;
    }
    
    Vec2 posCurrentAccel(const StageAccessor& stageAccessor) {
        const Chara& player = stageAccessor.player();
        float stopTurn = player.vel().length() / Parameter::CharaDecelSpeed();
        return posAfterTurn(stageAccessor, stopTurn);
    }
    
    // targetに現在のアクセルだけで到達可能かどうか
    bool isEnableReachInCurrentAccel(const StageAccessor& stageAccessor, Vec2 target, float radius)
    {
        const Chara& player = stageAccessor.player();
        // 何ターン後に止まるか
        float stopTurn = player.vel().length() / Parameter::CharaDecelSpeed();
        // 1ターンずつシミュレーションする
        for (int passedTurn = 1; passedTurn <= stopTurn; ++passedTurn) {
            Vec2 futurePos = posAfterTurn(stageAccessor, passedTurn);
            float futureDistance = (futurePos - target).length();
            if (futureDistance <= (Parameter::CharaRadius() + radius)) {
                return true;
            }
        }
        return false;
    }
    
    // 単純にハスの間の総距離を足して、全体の総距離を概算する
    float calcWholeDistance(const StageAccessor& aStageAccessor)
    {
        float distance = 0;
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        const Chara& player = aStageAccessor.player();
        
        int lotusesCount = lotuses.count();
        Vec2 current = player.pos();
        Vec2 firstTarget = lotuses[0].pos();
        // 最初の距離
        float initialDistance = (firstTarget - current).length();
        // 最後の距離
        float lastDistance = (lotuses[lotuses.count() - 1].pos() - lotuses[0].pos()).length();
        
        /*current = firstTarget;
         for (int i = 0; i < lotusesCount; ++i) {
         const Lotus& lotus = lotuses[i];
         const Lotus& next = lotuses[(i + 1) % lotuses.count()];
         distance += (next.pos() - lotus.pos()).length();
         }
         // 総距離の算出
         distance = distance * 3 + initialDistance;*/
        
        for (int i = 0; i < lotusesCount; ++i) {
            const Lotus& lotus = lotuses[i];
            const Lotus& next = lotuses[(i + 1) % lotuses.count()];
            distance += (next.pos() - lotus.pos()).length();
        }
        
        distance = distance * 3 + initialDistance - lastDistance;
        
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
        changedTarget = false;
        
        wholeDistance = 0;
        
        const Chara& player = aStageAccessor.player();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        initialPlayerPosition = player.pos();
        positionHistory[0] = player.pos();
        
        
        float v0 = Parameter::CharaAccelSpeed();
        float a = -1 * Parameter::CharaDecelSpeed();
        float stopTime = v0 / -a;
        float waitTurn = player.accelWaitTurn();
        
        // 予想最低速度を算出する
        int lotusCount = lotuses.count();
        float wd = calcWholeDistance(aStageAccessor);
        for (float apt = 1; apt < stopTime; ++apt) {
            float speed = v0 + a * (apt - 1);
            // 予想ターン数
            float estimateTurn = wd / speed;
            int accelCount = (estimateTurn / waitTurn) + player.accelCount();
            int requiredAccel = (estimateTurn / apt) + (lotusCount * 3 - 1);
            if (accelCount > requiredAccel) {
                minSpeed = speed;
                break;
            }
        }
        
        // 最後に通ったハス
        lastTargetLotusNo = player.targetLotusNo();
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
        //auto distance =
        // 進む距離
        int maxLotusCount = lotuses.count();
        
        // 最低限残しておくアクセル回数
        int saveAccelThreshold = 5;
        if (player.passedLotusCount() > maxLotusCount - 1) {
            // 最後の方は自重しなくする
            saveAccelThreshold = 1;
        }
        
        bool doAccel = false;
        // if (ac > 0) {
        
        Vec2 goal = getNextTarget(aStageAccessor);
        Vec2 sub = goal - player.pos();
        Vec2 vel = player.vel() + aStageAccessor.field().flowVel();
        const Lotus& targetLotus = lotuses[player.targetLotusNo()];
        
        float diffAngle = 0;
        if (vel.squareLength() > 0) {
            diffAngle = Math::Abs(Math::RadToDeg(vel.angle(sub)));
        }
        
        // 前回と目的地が変わってたら
        if (lastTargetLotusNo != player.targetLotusNo() && vel.length() <= minSpeed) {
            // そもそも速度が規定値以下なら踏む
            doAccel = true;
        } else {
            // アクセルを踏まずに将来的に移動しそうな点と目的地の距離 VS 今いる地点と目的地の距離を
            // 比べて、将来的に移動しそうな点の方が近ければ、少なくとも目的地の方向に動いているっぽいのでアクセルを踏まない
            float currentDitance = sub.squareLength();
            Vec2 futurePoint = posCurrentAccel(aStageAccessor);
            float futureDistance = (goal - futurePoint).squareLength();
            if (currentDitance < futureDistance) {
                doAccel = true;
            }
        }
        
        positionHistory[player.passedTurn()] = player.pos();
        
        // 予想最低速度を下回ってたらアクセルを踏む
        if (vel.length() < minSpeed) {
            doAccel = true;
        }
        
        if (changedTarget) {
            // ターゲットが変わってたらdoaccel
            doAccel = true;
        }
        
        lastTargetLotusNo = player.targetLotusNo();
        if (doAccel) {
            if (player.accelCount() > 0) {
                // これ以上加速しなくてもたどり着けそうなら勿体ないから加速しない
                if (!isEnableReachInCurrentAccel(aStageAccessor, targetLotus.pos(), targetLotus.radius())) {
                    changedTarget = false;
                    return Action::Accel(goal);
                }
            }
        }
        return Action::Wait();
    }
    
}

//------------------------------------------------------------------------------
// EOF
