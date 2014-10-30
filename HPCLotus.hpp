//------------------------------------------------------------------------------
/// @file
/// @brief    Lotus クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCCircle.hpp"

namespace hpc {

    class Stage;
    
    //------------------------------------------------------------------------------
    /// フィールド上に存在する1つの蓮をあらわします。
    class Lotus
    {
    public:
        Lotus();

        void reset(const Vec2& aPos, float aRadius);    ///< 情報を既定の値で初期化します。
        void reset();                                   ///< 情報を初期状態に戻します。

        Vec2 pos()const;                                ///< 蓮の位置を返します。
        float radius()const;                            ///< 蓮の半径を返します。
        const Circle& region()const;                    ///< 領域を表す円を返します。

    private:
        Circle mRegion;     ///< 位置情報
    };
}
//------------------------------------------------------------------------------
// EOF
