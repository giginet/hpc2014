//------------------------------------------------------------------------------
/// @file
/// @brief Timer クラス
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include <ctime>

namespace hpc {

    //------------------------------------------------------------------------------
    /// 実時間計測を行うタイマーを提供します。
    class Timer
    {
    public:
        Timer(int aLimitSec);               ///< 制限時間を定めてインスタンスを生成します。

        void start();                       ///< タイマーを開始します。
        bool isInTime()const;              ///< 制限時間内かどうかを返します。
        double pastSecForPrint()const;     ///< 表示用の経過時間を取得します。

    private:
        double pastSec()const;             ///< 経過時間を取得します。

        const int mLimitSec;                ///< 制限時間
        std::clock_t mTimeBegin;            ///< 開始時刻
    };
}
//------------------------------------------------------------------------------
// EOF
