//------------------------------------------------------------------------------
/// @file
/// @brief    Chara クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCAction.hpp"
#include "HPCBrain.hpp"
#include "HPCCircle.hpp"
#include "HPCStageAccessor.hpp"

namespace hpc {
    
    class CharaParam;
    class Random;
    
    //------------------------------------------------------------------------------
    /// キャラの情報を保持します。
    class Chara
    {
    public:
        Chara();

        void init(const Stage& aStage, int aCharaIndex);    ///< 準備処理を行います。
        void decideAction(Random& aRandom);                 ///< 動作を決定します。
        void execAction();                                  ///< 動作を実行します。
        void move();                                        ///< 移動処理を行います。
        void separation(const Vec2& aSeparateVec);          ///< めり込み補正を行います。
        void updateTurn();                                  ///< ターン経過処理を行います。
        void correctInside();                               ///< フィールドの内側に補正します。
        
        void reset();                                       ///< 状態をリセットします。
        void setup(const Vec2& aPos, const CharaParam& aCharaParam); ///< 初期設定を行います。
        void incTargetLotusNo();                            ///< 次に目指す蓮の番号を１つ進めます。
        void setVel(const Vec2& aVel);                      ///< 速度を設定します。
        void setRank(int aRank);                            ///< 順位を設定します。

        const Circle& region()const;                        ///< 領域を表す円を返します。
        Vec2 pos()const;                                    ///< 現在位置を返します。
        Vec2 vel()const;                                    ///< 現在速度を返します。
        bool isGoal()const;                                 ///< ゴールしたかどうかを返します。
        int accelCount()const;                              ///< 加速できる回数を返します。
        int accelWaitTurn()const;                           ///< 加速回数が増えるまでの残りターン数を返します。
        int targetLotusNo()const;                           ///< 次に目指す蓮の番号を返します。
        int roundCount()const;                              ///< 周回数を返します。
        int rank()const;                                    ///< 順位を返します。
        int passedLotusCount()const;                        ///< 通過した蓮の数を返します。
        int passedTurn()const;                              ///< 経過ターン数を返します。
        
        const Circle& prevRegion()const;                    ///< 前回領域を表す円を返します。

    private:
        StageAccessor mStageAccessor;   ///< ステージ情報のアクセサ
        Brain mBrain;                   ///< 動作決定モジュール
        Circle mRegion;                 ///< 領域
        Circle mPrevRegion;             ///< 前回領域
        Action mDecidedAction;          ///< 決定された動作
        Vec2 mVel;                      ///< 速度
        int mAccelCount;                ///< 加速できる回数
        int mAccelWaitTurn;             ///< 加速回数が増えるまでの残りターン数
        int mAccelWaitTurnMax;          ///< 加速回数が増えるまでの残りターン数 の設定値
        int mTargetLotusNo;             ///< 現在の目指す蓮番号
        int mRoundCount;                ///< 周回数
        int mRank;                      ///< 順位
        int mPassedTurn;                ///< 経過ターン数
        
        void accelIfPossible(const Vec2& aTargetPos);       ///< 加速できるなら加速します。
    };
}
//------------------------------------------------------------------------------
// EOF
