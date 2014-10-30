//------------------------------------------------------------------------------
/// @file
/// @brief    LevelDesigner クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCRandom.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ステージごとのマップ生成機能を提供します。
    class LevelDesigner
    {
    public:
        /// ステージのマップを生成します。
        static void Setup(int aNumber, Stage& aStage, Random& aRandom);

    private:
        LevelDesigner();
    };
}
//------------------------------------------------------------------------------
// EOF
