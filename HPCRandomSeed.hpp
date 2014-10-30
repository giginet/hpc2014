//------------------------------------------------------------------------------
/// @file
/// @brief RandomSeed クラス
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCTypes.hpp"

namespace hpc {
    
    //------------------------------------------------------------------------------
    /// 乱数のシードを表します。
    class RandomSeed
    {
    public:
        RandomSeed();
        RandomSeed(uint x, uint y, uint z, uint w);

        uint x;
        uint y;
        uint z;
        uint w;
    };
}
//------------------------------------------------------------------------------
// EOF
