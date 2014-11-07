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
    
    /// シミュレーション用のダミープレイヤーです
    struct DummyPlayer
    {
        int roundCount;
        int passedTurn;
        int passedLotusCount;
        int accelCount;
        int accelWaitTurn;
        int targetLotusNo;
        Vec2 pos;
        Vec2 vel;
    };
    /// プレイヤーの初期位置
    Vec2 _initialPlayerPosition;
    /// フィールド（グローバルにアクセスできるようにしている）
    Field _field;
    /// ハスのリスト（グローバルにアクセスできるようにしている）
    LotusCollection _lotuses;
    /// 予想最低速度
    float _minSpeed = 0;
    
    int _lastTargetLotusNo = -1;
    
    /// 最後にアクセルを踏んだ地点
    Vec2 _lastAccelPos;
    float _lastAccelTurn;
    
    /// 過去の移動履歴
    Vec2 _positionHistory[Parameter::GameTurnPerStage];
    
    
    /// デバッグ用
    int _stageNo = 0;
}

/// プロコン問題環境を表します。
namespace hpc {
    
    /// 新しいダミー用のプレイヤーを生成します
    DummyPlayer createDummyPlayer()
    {
        DummyPlayer dummy;
        dummy.roundCount = 0;
        dummy.passedTurn = 0;
        dummy.passedLotusCount = 0;
        dummy.accelCount = 0;
        dummy.accelWaitTurn = Parameter::CharaAddAccelWaitTurn;
        dummy.targetLotusNo = 0;
        dummy.pos = _initialPlayerPosition;
        dummy.vel = Vec2();
        return dummy;
    }
    
    /// Charaクラスからダミー用のプレイヤーを生成します
    DummyPlayer createDummyPlayer(const Chara& player)
    {
        DummyPlayer dummy;
        dummy.roundCount = player.roundCount();
        dummy.passedTurn = player.passedTurn();
        dummy.passedLotusCount = player.passedLotusCount();
        dummy.accelCount = player.accelCount();
        dummy.accelWaitTurn = player.accelWaitTurn();
        dummy.targetLotusNo = player.targetLotusNo();
        dummy.pos = player.pos();
        dummy.vel = player.vel();
        return dummy;
    }
    
    /// ハスa, b, cを通るときに、一番いい感じでbに接触できるような点を返します
    Vec2 getTargetByThreePoints(const Lotus& target, Vec2 prevPoint, Vec2 nextPoint)
    {
        // acベクトルを生成
        Vec2 ac = nextPoint - prevPoint;
        
        // acベクトルを90度回転する
        ac.rotate(Math::DegToRad(90));
        
        ac.normalize(target.radius() * 0.75);
        
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
    
    /// ハスa, bを通るときに、一番いい感じでaに接触できるような点を返します
    Vec2 getTargetByTwoPoints(const Lotus& target, Vec2 nextPoint)
    {
        // abベクトルを生成
        Vec2 ab = nextPoint - target.pos();
        ab.normalize(target.radius() * 0.75);
        
        Vec2 goal = target.pos() + ab;
        return goal;
    }
    
    /// 次の目的地を返します
    Vec2 getNextTarget(DummyPlayer player)
    {
        const float v0 = Parameter::CharaAccelSpeed();
        const float d = -Parameter::CharaDecelSpeed();
        float t = -(v0 / d);
        int lotusCount = _lotuses.count();
        
        int targetLotusNo = player.targetLotusNo;
        int roundNo = player.roundCount;
        // もし、targetが最後ハスだったら
        if (roundNo == 2 && targetLotusNo == lotusCount - 1)
        {
            const Lotus& lotus = _lotuses[targetLotusNo];
            Vec2 sub = player.pos - lotus.pos();
            sub.normalize(lotus.radius() * 0.75);
            return lotus.pos() + sub;
        } else {
            Vec2 goal;
            
            // それ以外の時
            // 1つ先のゴールを出す
            Vec2 prevPoint;
            if (roundNo == 0 && targetLotusNo == 0) {
                // まだ1つも通ってないとき、最初の点を使う
                prevPoint = _initialPlayerPosition;
            } else {
                // それ以外の時、1個前の点を使う
                prevPoint = _lotuses[(targetLotusNo - 1 + lotusCount) % lotusCount].pos();
            }
            const Lotus& target = _lotuses[targetLotusNo];
            const Lotus& target2 = _lotuses[(targetLotusNo + 1) % lotusCount];
            Vec2 goalA0 = getTargetByThreePoints(target, prevPoint, target2.pos());
            Vec2 goalB0 = getTargetByTwoPoints(target, target2.pos());
            
            // 2つ先のゴールを出す
            Vec2 nextPoint2 =_lotuses[(targetLotusNo + 2) % lotusCount].pos(); // 2つあと
            Vec2 goalA1 = getTargetByThreePoints(target2, target.pos(), nextPoint2);
            Vec2 goalB1 = getTargetByTwoPoints(target2, nextPoint2);
            if ((goalA0 + goalA1).squareLength() < (goalB0 + goalB1).squareLength()) {
                // Aルートの方が近い
                goal = goalA0;
            } else {
                // Bルートの方が近い
                goal = goalB0;
            }
            
            
            Vec2 stream = _field.flowVel();
            goal -= stream * t;
            return goal;
        }
        
    }
    
    /// nターン後の位置を返す
    Vec2 posAfterTurn(DummyPlayer dplayer, int afterTurn) {
        float stopTurn = dplayer.vel.length() / Parameter::CharaDecelSpeed();
        if (afterTurn > stopTurn)
        {
            afterTurn = stopTurn;
        }
        float a = -Parameter::CharaDecelSpeed();
        Vec2 currentVel = dplayer.vel;
        Vec2 currentPos = dplayer.pos;
        for (int passedTurn = 1; passedTurn <= afterTurn; ++passedTurn) {
            currentPos = currentPos + currentVel + _field.flowVel();
            float currentSpeed = dplayer.vel.length();
            currentVel.normalize();
            currentSpeed += a;
            currentVel *= currentSpeed;
        }
        return currentPos;
    }
    
    Vec2 posCurrentAccel(DummyPlayer dplayer) {
        float stopTurn = dplayer.vel.length() / Parameter::CharaDecelSpeed();
        return posAfterTurn(dplayer, stopTurn);
    }
    
    // targetに現在のアクセルだけで到達可能かどうか
    bool isEnableReachInCurrentAccel(DummyPlayer dplayer, Vec2 target, float radius)
    {
        Vec2 prevPos = dplayer.pos;
        // 何ターン後に止まるか
        float stopTurn = dplayer.vel.length() / Parameter::CharaDecelSpeed();
        float charaRadius = Parameter::CharaRadius();
        // 1ターンずつシミュレーションする
        for (int passedTurn = 1; passedTurn <= stopTurn; ++passedTurn) {
            Vec2 futurePos = posAfterTurn(dplayer, passedTurn);
            if (Collision::IsHit(Circle(target, radius), Circle(prevPos, charaRadius), futurePos)) {
                return true;
            }
            prevPos = futurePos;
        }
        return false;
    }
    
    /// GetNextActionをダミープレイヤーでシミュレーションする
    Action simulateGetNextAction(DummyPlayer dplayer, float minSpeed)
    {
        // 最低限残しておくアクセル回数
        bool saveAccel = true;
        if (dplayer.targetLotusNo >= _lotuses.count() - 1 && dplayer.roundCount == 2) {
            // 最後の方は自重しなくする
            saveAccel = false;
        }
        
        bool doAccel = false;
        
        Vec2 goal = getNextTarget(dplayer);
        Vec2 sub = goal - dplayer.pos;
        Vec2 vel = dplayer.vel + _field.flowVel();
        const Lotus& targetLotus = _lotuses[dplayer.targetLotusNo];
        
        float diffAngle = 0;
        if (vel.squareLength() > 0) {
            diffAngle = Math::Abs(Math::RadToDeg(vel.angle(sub)));
        }
        
        // 前回と目的地が変わってたら
        if (vel.length() <= minSpeed) {
            // そもそも速度が規定値以下なら踏む
            doAccel = true;
        } else {
            // アクセルを踏まずに将来的に移動しそうな点と目的地の距離 VS 今いる地点と目的地の距離を
            // 比べて、将来的に移動しそうな点の方が近ければ、少なくとも目的地の方向に動いているっぽいのでアクセルを踏まない
            // 最後にアクセルを踏んでから3ターン経過してなければ勿体ないから踏まない
            float currentDitance = sub.squareLength();
            Vec2 futurePoint = posCurrentAccel(dplayer);
            float futureDistance = (goal - futurePoint).squareLength();
            if (currentDitance < futureDistance) {
                doAccel = true;
            }
        }
        
        _lastTargetLotusNo = dplayer.targetLotusNo;
        _positionHistory[dplayer.passedTurn] = dplayer.pos;
        
        if (doAccel) {
            if (dplayer.accelCount > 0) {
                // アクセルを節約しているとき、これ以上加速しなくてもたどり着けそうなら勿体ないから加速しない
                if (!saveAccel || !isEnableReachInCurrentAccel(dplayer, targetLotus.pos(), targetLotus.radius())) {
                    _lastAccelTurn = dplayer.passedTurn;
                    _lastAccelPos = dplayer.pos;
                    return Action::Accel(goal);
                }
            }
        }
        return Action::Wait();
    }
    
    //------------------------------------------------------------------------------
    /// 各ステージ開始時に呼び出されます。
    ///
    /// この関数を実装することで、各ステージに対して初期処理を行うことができます。
    ///
    /// @param[in] aStageAccessor 現在のステージ。
    void Answer::Init(const StageAccessor& aStageAccessor)
    {
        const Chara& player = aStageAccessor.player();
        _initialPlayerPosition = player.pos();
        _positionHistory[0] = player.pos();
        // fieldとlotusesは変更され得ないので、最初にコピーしてグローバルにアクセスできるようにしている
        _field = aStageAccessor.field();
        _lotuses = aStageAccessor.lotuses();
        
        // 予想最低速度を算出する
        float minPassedTurn = Parameter::GameTurnPerStage;
        float minSpeed = Parameter::CharaAccelSpeed();
        
        // minSpeedを徐々に変えてって一番早く回れた奴を採用する
        for (float speed = Parameter::CharaAccelSpeed(); speed >= Parameter::CharaDecelSpeed() * 2; speed -= 0.01) {
            // めっちゃリアルっぽいシミュレーションする
            DummyPlayer dummyPlayer = createDummyPlayer(player);
            // ゴールするまでリアルシミュレーション
            // 経験上、2500ターンは超えない気がするから2500まで
            for (int passedTurn = 0; passedTurn <= 2500; ++passedTurn) {
                const Lotus& targetLotus = _lotuses[dummyPlayer.targetLotusNo];
                Action nextAction = simulateGetNextAction(dummyPlayer, speed);
                if (nextAction.type() == ActionType_Accel && dummyPlayer.accelCount > 0) {
                    // アクセルを踏む
                    const Vec2 toTargetVec = nextAction.value() - dummyPlayer.pos;
                    // 目標座標とキャラ座標が同値の場合、何もしない
                    if (!toTargetVec.isZero()) {
                        --dummyPlayer.accelCount;
                        dummyPlayer.vel = toTargetVec.getNormalized(Parameter::CharaAccelSpeed());
                    }
                } else {
                    // Waitなら何もしない
                }
                Vec2 prevPos = dummyPlayer.pos;
                dummyPlayer.pos += dummyPlayer.vel;
                dummyPlayer.pos += _field.flowVel();
                
                // 減速させる
                if (!dummyPlayer.vel.isZero()) {
                    const float len = Math::Max(
                                                dummyPlayer.vel.length() - Parameter::CharaDecelSpeed()
                                                , 0.0f
                                                );
                    if (0.0f < len) {
                        dummyPlayer.vel.normalize(len);
                    } else {
                        dummyPlayer.vel.reset();
                    }
                }
                if (Collision::IsHit(targetLotus.region(), Circle(prevPos, Parameter::CharaRadius()), dummyPlayer.pos)) {
                    // 目標の蓮を通過したら、次の蓮との判定を行う
                    ++dummyPlayer.targetLotusNo;
                    // 一周回ったら周回数加算
                    if (_lotuses.count() == dummyPlayer.targetLotusNo) {
                        dummyPlayer.targetLotusNo = 0;
                        ++dummyPlayer.roundCount;
                    }
                }
                // ゴールしてたら探索終了
                if (dummyPlayer.roundCount == Parameter::StageRoundCount)
                {
                    if (passedTurn < minPassedTurn) {
                        // 最速だったら記録
                        minPassedTurn = passedTurn;
                        minSpeed = speed;
                    }
                    break;
                }
                if (passedTurn >= minPassedTurn) {
                    // 経過ターン数が今までの最低値を上回ったらもうチェックする意味ないので省略
                    break;
                }
                // プレイヤーの更新
                dummyPlayer.passedTurn = passedTurn;
                --dummyPlayer.accelWaitTurn;
                if (dummyPlayer.accelWaitTurn <= 0) {
                    dummyPlayer.accelCount = Math::Min(dummyPlayer.accelCount + 1, Parameter::CharaAccelCountMax);
                    dummyPlayer.accelWaitTurn = Parameter::CharaAddAccelWaitTurn;
                }
            }
        }
        _minSpeed = minSpeed;
        
        //std::cout << "StageNO: " <<  _stageNo << std::endl;
        //std::cout << "minSpeed: " << minSpeed << std::endl;
        //std::cout << "estimateTurn " << minPassedTurn << std::endl;
        
        // シミュレーション後にグローバル変数を元に戻す
        _lastAccelTurn = 0;
        _lastTargetLotusNo = -1;
        _lastAccelPos = Vec2();
        
        // 最後に通ったハス
        ++_stageNo;
    }
    
    //------------------------------------------------------------------------------
    /// 各ターンでの動作を返します。
    ///
    /// @param[in] aStageAccessor 現在ステージの情報。
    ///
    /// @return これから行う動作を表す Action クラス。
    Action Answer::GetNextAction(const StageAccessor& aStageAccessor)
    {
        DummyPlayer dplayer = createDummyPlayer(aStageAccessor.player());
        return simulateGetNextAction(dplayer, _minSpeed);
    }
    
}

//------------------------------------------------------------------------------
// EOF
