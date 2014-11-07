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
    float accelPerTurn = 0;
    
    float minSpeed = 0;
    
    // 最後にアクセルを踏んだ地点
    Vec2 lastAccellPos;
    
    // 過去の移動履歴
    Vec2 positionHistory[Parameter::GameTurnPerStage];
    
    // デバッグ用（あとで消す）
    int stageNo = 0;
    float realDistance = 0;
    
}

/// プロコン問題環境を表します。
namespace hpc {
    
    // 加速してからtターン後の位置を返します
    float distanceAfterTurn(float t, const StageAccessor& aStageAccessor)
    {
        float v0 = Parameter::CharaAccelSpeed();
        float a = -Parameter::CharaDecelSpeed();
        if (t >= v0 / -a) {
            t = v0 / -a;
        }
        return v0 * t + 0.5 * a * t * t;
    }
    
    /// ハスa, b, cを通るときに、一番いい感じでbに接触できるような点を返します
    Vec2 getTargetByThreePoints(const Lotus& target, Vec2 prevPoint, Vec2 nextPoint)
    {
        // acベクトルを生成
        Vec2 ac = nextPoint - prevPoint;
        
        // acベクトルを90度回転する
        ac.rotate(Math::DegToRad(90));
        
        ac.normalize();
        
        ac *= target.radius() * 0.9;
        
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
            return lotus.pos() + sub * lotus.radius();
        } else {
            Vec2 goal;
            
            // それ以外の時
            // 1つ先のゴールを出す
            Vec2 prevPoint;
            if (roundNo == 0 && targetLotusNo == 0) {
                prevPoint = initialPlayerPosition;
            } else {
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
        if ( (player.pos() - target).squareLength() < Parameter::CharaRadius() ) {
            return true;
        }
        Vec2 goal = canReachCurrentAccelPos(player);
        return (goal - target).squareLength() <= Parameter::CharaRadius();
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
    
    // 渡されたターン数連続で指定座標まで遠ざかっていればtrueを返します
    bool isFartherPastTurn(int turn, int currentTurn, Vec2 targetPos)
    {
        for (int i = turn; i > 1; --i) {
            Vec2 prevPos = positionHistory[currentTurn - i - 1];
            Vec2 currentPos = positionHistory[currentTurn - i];
            if ((targetPos - prevPos).squareLength() < (targetPos - currentPos).squareLength()) {
                return false;
            }
        }
        return true;
    }
    
    //------------------------------------------------------------------------------
    /// 各ステージ開始時に呼び出されます。
    ///
    /// この関数を実装することで、各ステージに対して初期処理を行うことができます。
    ///
    /// @param[in] aStageAccessor 現在のステージ。
    void Answer::Init(const StageAccessor& aStageAccessor)
    {
        realDistance = 0;
        changedTarget = false;
        
        wholeDistance = 0;
        accelPerTurn = 0;
        
        const Chara& player = aStageAccessor.player();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
        initialPlayerPosition = player.pos();
        positionHistory[0] = player.pos();
        
        
        float v0 = Parameter::CharaAccelSpeed();
        float a = -1 * Parameter::CharaDecelSpeed();
        float stopTime = v0 / -a;
        float waitTurn = player.accelWaitTurn();
        
        // 平均速度を算出する
        int lotusCount = lotuses.count();
        float wd = calcWholeDistance(aStageAccessor);
        for (float apt = 1; apt < stopTime; ++apt) {
            // 予想ターン数
            float speed = v0 + a * (apt - 1);
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
        float diffAngle = 0;
        if (vel.squareLength() > 0) {
            diffAngle = Math::Abs(Math::RadToDeg(vel.angle(sub)));
        }
        
        // 前回と目的地が変わってたらアクセル踏み直す
        if (lastTargetLotusNo != player.targetLotusNo()) {
            doAccel = true;
            changedTarget = true;
        } else if (vel.length() < minSpeed && player.accelCount() >= saveAccelThreshold && player.passedTurn() >= 3) {
            // 過去3ターンの履歴を見て、徐々に遠ざかってたら再アクセルを踏む
            float currentDistance = (goal - player.pos()).squareLength();
            float prevDistance = (goal - positionHistory[player.passedTurn() - 1]).squareLength();
            float prevDistance2 = (goal - positionHistory[player.passedTurn() - 2]).squareLength();
            float prevDistance3 = (goal - positionHistory[player.passedTurn() - 3]).squareLength();
            if (currentDistance > prevDistance && prevDistance > prevDistance2 && prevDistance2 > prevDistance3) {
                doAccel = true;
            }
        }
        
        positionHistory[player.passedTurn()] = player.pos();
        
        // 予想最低速度を下回ってたらアクセルを踏む
        if (vel.length() < minSpeed) {
            float stopTurn = player.vel().length() / Parameter::CharaDecelSpeed();
            float v0 = player.vel().length();
            float a = -Parameter::CharaDecelSpeed();
            float length = v0 * stopTurn + 0.5 * a * stopTurn * stopTurn;
            // これ以上加速しなくてもたどり着けそうなら勿体ないから加速しない
            if (sub.length() >= (length + Parameter::CharaRadius())) {
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
                changedTarget = false;
                return Action::Accel(goal);
            }
        }
        return Action::Wait();
    }
    
}

//------------------------------------------------------------------------------
// EOF
