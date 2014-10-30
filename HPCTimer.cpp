//------------------------------------------------------------------------------
/// @file
/// @brief HPCTimer.hpp の実装
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCTimer.hpp"

namespace {

    //------------------------------------------------------------------------------
    /// 現在の時間を取得します。
    ///
    /// @return 現在の時刻を表す std::clock_t 型オブジェクト。
    std::clock_t GetCurrentTime()
    {
        return ::std::clock();
    }

    //------------------------------------------------------------------------------
    /// std::clock_t で表される時間を秒に変換します。
    ///
    /// @param[in] aTime std::clock_t 型で表される時間
    ///
    /// @return aTime を秒に変換した値。
    double ToSec(std::clock_t aTime)
    {
        return static_cast<double>(aTime) / CLOCKS_PER_SEC;
    }
}

namespace hpc {

    //------------------------------------------------------------------------------
    /// 制限時間を aLimitSec [秒] としてタイマーのインスタンスを生成します。
    ///
    /// @note 生成しただけでは計測を行いません。
    ///       計測を行うには start 関数を呼び出します。
    ///
    /// @param[in] aLimitSec 制限時間を秒で指定。
    Timer::Timer(int aLimitSec)
        : mLimitSec(aLimitSec)
        , mTimeBegin(std::clock_t())
    {
    }

    //------------------------------------------------------------------------------
    /// タイマーの計測を開始します。
    void Timer::start()
    {
        mTimeBegin = GetCurrentTime();
    }

    //------------------------------------------------------------------------------
    /// start 関数を呼び出した時点からの経過時間を取得します。
    ///
    /// @return start を呼び出してからの経過時間を秒に変換したもの。
    double Timer::pastSec()const
    {
        return ToSec(GetCurrentTime() - mTimeBegin);
    }

    //------------------------------------------------------------------------------
    /// 表示用に修正された時間を表示します。
    ///
    /// pastSec と異なり、制限時間を超過した場合は制限時間を返します。
    ///
    /// @return start を呼び出してからの時間を秒に修正し、表示用に調整したもの。
    double Timer::pastSecForPrint()const
    {
        if (!isInTime()) {
            return mLimitSec;
        } else {
            return pastSec();
        }
    }

    //------------------------------------------------------------------------------
    /// @return 制限時間以内の場合 @c false を返し、
    ///         超過した場合は @c true を返します。
    bool Timer::isInTime()const
    {
        return pastSec() < mLimitSec;
    }
}
//------------------------------------------------------------------------------
// EOF
