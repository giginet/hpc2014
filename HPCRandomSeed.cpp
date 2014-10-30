//------------------------------------------------------------------------------
/// @file
/// @brief HPCRandomSeed.hpp の実装
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCRandomSeed.hpp"

//------------------------------------------------------------------------------
namespace {
    /// @name シードの初期値
    ///
    /// @note シードの初期値は途中評価用です。
    ///       最終評価では異なるシードによる評価が行われます。
    ///       このシードの各要素の値を変えることで、プログラムが
    ///       いろいろなシードの値に耐えうるかを調べることができます。
    //@{
    const uint DefaultSeedX = 2942096179u;
    const uint DefaultSeedY = 2745714780u;
    const uint DefaultSeedZ = 3684690907u;
    const uint DefaultSeedW = 3549078838u;
    //@}
}

namespace hpc {
    
    //------------------------------------------------------------------------------
    RandomSeed::RandomSeed()
        : x(DefaultSeedX)
        , y(DefaultSeedY)
        , z(DefaultSeedZ)
        , w(DefaultSeedW)
    {
    }

    //------------------------------------------------------------------------------
    RandomSeed::RandomSeed(uint aX, uint aY, uint aZ, uint aW)
        : x(aX)
        , y(aY)
        , z(aZ)
        , w(aW)
    {
    }

}
//------------------------------------------------------------------------------
// EOF
