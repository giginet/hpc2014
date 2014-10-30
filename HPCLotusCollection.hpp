//------------------------------------------------------------------------------
/// @file
/// @brief    LotusCollection クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCLotus.hpp"
#include "HPCParameter.hpp"
#include "HPCVec2.hpp"

namespace hpc {

    class Stage;
    
    //------------------------------------------------------------------------------
    /// 蓮の組を表します。
    ///
    /// 1ステージのすべての蓮は、この LotusCollection クラスに
    /// 格納されます。
    class LotusCollection
    {
    public:
        LotusCollection();

        void set(const LotusCollection& aRhs);              ///< 蓮データを設定します。
        void reset();                                       ///< 蓮データを初期化します。
        void setupAddLotus(const Vec2& aPos, float aRadius);///< 蓮を追加します。

        int count()const;                                   ///< 有効な蓮数を返します。

        /// @name 有効な蓮へのアクセス
        //@{
        const Lotus& operator[](int aIndex)const;
        Lotus& operator[](int aIndex);
        //@}

    private:
        Lotus mLotuses[Parameter::LotusCountMax];   ///< 蓮用配列
        int mCount;                                 ///< 蓮数
    };
}
//------------------------------------------------------------------------------
// EOF
