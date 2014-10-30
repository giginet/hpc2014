//------------------------------------------------------------------------------
/// @file
/// @brief RandomSet クラス
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCRandom.hpp"
#include "HPCRandomSeed.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 乱数生成クラスの集合を表します。
    class RandomSet
    {
    public:
        explicit RandomSet(const RandomSeed& aSeed = RandomSeed());

        /// @name 各要素へのアクセス
        //@{
        Random& system();               ///< システムで使用するインスタンスを返します。
        Random& game();                 ///< ゲーム中に使用するインスタンスを返します。
        //@}

    private:
        Random mSystem;         ///< システムで使用する乱数生成クラス
        Random mGame;           ///< ゲーム中に使用する乱数生成クラス
    };
}
//------------------------------------------------------------------------------
// EOF
