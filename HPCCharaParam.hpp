//------------------------------------------------------------------------------
/// @file
/// @brief    CharaParam クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCCharaType.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// キャラのパラメータを表します。
    ///
    /// CharaType が Cpu の場合のみ、強さパラメータが有効になります。
    class CharaParam
    {
    public:
        static const CharaParam CreateHuman();              ///< Human用のインスタンスを返します。
        /// Cpu用のインスタンスを返します。
        static const CharaParam CreateCpu(int aStrength);
        
    public:
        /// インスタンスを生成します。
        CharaParam();

        void reset();                           ///< 値を初期化します。
        
        CharaType type()const;                  ///< キャラの種類を返します。
        
        /// @name パラメータへのアクセス
        //@{
        int strength()const;
        //@}

    private:
        CharaType mType;    ///< キャラの種類
        int mStrength;      ///< 強さ
    };
}
//------------------------------------------------------------------------------
// EOF
