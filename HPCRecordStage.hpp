//------------------------------------------------------------------------------
/// @file
/// @brief    RecordStage クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCField.hpp"
#include "HPCParameter.hpp"
#include "HPCStage.hpp"
#include "HPCTurnResult.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief 各ステージの記録を表します。
    class RecordStage 
    {
    public:
        RecordStage();

        void writeStart(const Stage& aStage);               ///< 記録を開始します。
        void writeTurn(const TurnResult& aResult);          ///< 各ターンの内容を記録します。
        void writeEnd(const Stage& aStage);                 ///< 終了時の内容を記録します。

        double score()const;                               ///< ステージ毎の得点を返します。
        void dump()const;                                  ///< 実行結果を画面に表示します。
        void dumpJson(bool aIsCompressed)const;            ///< 実行結果を JSON 形式で画面に表示します。

    private:
        int mCurrentTurn;                                   ///< 現在のターン番号
        int mRanks[Parameter::CharaCountMax];               ///< 順位
        int mPassedLotusCount;                              ///< 通過した蓮の数
        int mCharaCount;                                    ///< キャラ数
        
        // 詳細な記録は、定数 DEBUG が定義されている場合にのみ表示されます。
#ifdef DEBUG
        TurnResult mTurns[Parameter::GameTurnPerStage + 1]; ///< 記録するターン。初期状態を含めるので1多くとる。
        Field mField;                                       ///< フィールド情報
        LotusCollection mLotuses;                           ///< 蓮情報
        Vec2 mInitPositions[Parameter::CharaCountMax];      ///< 開始位置
#endif
        bool mIsFailed;     ///< ステージ途中で失敗したか
    };
}
//------------------------------------------------------------------------------
// EOF
