//------------------------------------------------------------------------------
/// @file
/// @brief    Record クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCRecordStage.hpp"
#include "HPCStage.hpp"
#include "HPCTurnResult.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ゲームの記録を保持します。
    class Record 
    {
    public:
        Record();

        /// @name 記録動作を行う関数
        //@{
        void writeStartStage(int aStageIndex, const Stage& aStage); ///< ステージの記録を開始します。
        void writeTurn(const TurnResult& aResult);                  ///< 各ターンの結果を記録します。
        void writeEndStage(const Stage& aStage);                    ///< 終了時の結果を記録します。
        //@}

        /// @name 記録を読み出す関数
        //@{
        int score()const;                                  ///< 合計得点を取得します。
        void dumpStage(int aStageIndex)const;              ///< ステージの結果を出力します。
        void dumpJsonStage(int aStageIndex)const;          ///< ステージの結果を JSON で出力します。
        void dumpJson(bool isCompressed)const;             ///< 全結果を JSON で出力します。
        //@}

    private:
        RecordStage mStage[Parameter::GameStageCount];    ///< ステージごとのデータ
        int mCurrentStageIndex;                             ///< 現在のステージ番号
    };
}
//------------------------------------------------------------------------------
// EOF
