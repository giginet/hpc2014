//------------------------------------------------------------------------------
/// @file
/// @brief    Field クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCRectangle.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 矩形のフィールドを表します。
    ///
    /// フィールドには「流れ」があり、キャラに影響します。
    /// 蓮には影響しません。
    class Field
    {
    public:
        Field();

        /// 初期設定を行います。
        void setup(
            const Rectangle& aRect
            , const Vec2& aFlowSpeed
            );
        void set(const Field& aField);              ///< フィールド情報を設定します。
        void reset();                               ///< 初期化します。
        
        const Rectangle& rect()const;               ///< 矩形を返します。
        const Vec2& flowVel()const;                 ///< 流れる速度を返します。

    private:
        Rectangle mRect;    ///< 矩形
        Vec2 mFlowVel;      ///< 流れる速度
    };
}
//------------------------------------------------------------------------------
// EOF
