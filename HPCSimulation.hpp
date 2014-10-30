//------------------------------------------------------------------------------
/// @file
/// @brief    Simulation クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCGame.hpp"
#include "HPCRandomSet.hpp"
#include "HPCTimer.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief ゲームのシミュレーションを表します。
    class Simulation 
    {
    public:
        Simulation();

        void run();                                    ///< 開始する
        void debug();                                  ///< デバッグする
        void outputResult()const;                     ///< 結果を表示する。
        void outputJson(bool isCompressed)const;      ///< JSON の出力を行う。
        
    private:
        RandomSet mRandSet; ///< 乱数生成クラス
        Game mGame;         ///< シミュレーションするゲーム
        Timer mTimer;       ///< ゲームタイマー

        void runDebugger();
    };
}
//------------------------------------------------------------------------------
// EOF
