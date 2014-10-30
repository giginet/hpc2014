//------------------------------------------------------------------------------
/// @file
/// @brief    Game クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCParameter.hpp"
#include "HPCRandomSet.hpp"
#include "HPCRecord.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ゲーム全体を表します。
    class Game 
    {
    public:
        Game(RandomSet& aRandSet);

        void startStage();                  ///< 現在のステージを開始します。
        void runTurn();                     ///< 現在実行中のステージでターンを1つ進めます。
        StageState state()const;           ///< ステージ内での現在の状態を表します。
        void onStageDone();                 ///< ステージ終了を通知します。
        bool isValidStage()const;          ///< 現在のステージが有効なものかどうかを返します。

        const Record& record()const;       ///< 記録へのアクセサ

    private:
        RandomSet& mRandSet;                ///< 乱数生成
        Stage mStage;                       ///< ステージ
        int mCurrentStageIndex;             ///< 現在のステージ番号
        Record mRecord;                     ///< 記録
    };
}
//------------------------------------------------------------------------------
// EOF
