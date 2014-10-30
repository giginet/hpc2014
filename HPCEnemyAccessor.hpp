//------------------------------------------------------------------------------
/// @file
/// @brief    EnemyAccessor クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCParameter.hpp"
#include "HPCVec2.hpp"

namespace hpc {

    class Chara;
    class CharaCollection;
    
    //------------------------------------------------------------------------------
    /// 敵キャラへのアクセスを提供します。
    ///
    /// 1ステージのすべての敵キャラは、このクラスからアクセスできます。
    /// (ゴールしているキャラも含みます)
    /// 実体はCharaCollectionに存在します。
    class EnemyAccessor
    {
    public:
        EnemyAccessor();

        void init(const CharaCollection& aCharas, int aPlayerIndex);  ///< 初期設定を行います。
        
        int count()const;                               ///< 有効な敵キャラ数を返します。
        const Chara& operator[](int aIndex)const;       ///< 有効な敵キャラへの参照を返します。

    private:
        const Chara* mEnemies[Parameter::CharaCountMax - 1]; ///< 敵キャラのポインタ配列
        int mCount;                                         ///< 有効な敵キャラ数
    };
}
//------------------------------------------------------------------------------
// EOF
