//------------------------------------------------------------------------------
/// @file
/// @brief    HPCEnemyAccessor.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCEnemyAccessor.hpp"

#include "HPCCharaCollection.hpp"
#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    EnemyAccessor::EnemyAccessor()
        : mEnemies()
        , mCount(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 初期設定を行います。
    ///
    /// @param[in] aCharas      CharaCollection クラスへの参照。
    /// @param[in] aPlayerIndex プレイヤーのインデックス。
    void EnemyAccessor::init(const CharaCollection& aCharas, int aPlayerIndex)
    {
        mCount = 0;
        
        // プレイヤー以外のキャラを敵キャラとして配列に保持します。
        for (int index = 0; index < aCharas.count(); ++index) {
            if (index == aPlayerIndex) {
                continue;
            }
            
            mEnemies[mCount] = &aCharas[index];
            ++mCount;
        }
        
        // 敵キャラ数 ＝ キャラ総数 - 1 である事を確認
        HPC_ASSERT(mCount == aCharas.count() - 1);
    }

    //------------------------------------------------------------------------------
    /// @return 現在有効な敵キャラ数。
    int EnemyAccessor::count()const
    {
        return mCount;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex 敵キャラのインデックス。
    ///
    /// @return aIndex 番目の敵キャラへの参照を返します。
    const Chara& EnemyAccessor::operator[](int aIndex)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return *(mEnemies[aIndex]);
    }
}

//------------------------------------------------------------------------------
// EOF
