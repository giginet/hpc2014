//------------------------------------------------------------------------------
/// @file
/// @brief    HPCLotusCollection.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCLotusCollection.hpp"

#include "HPCCommon.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    LotusCollection::LotusCollection()
        : mLotuses()
        , mCount(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 蓮のデータを設定します。
    /// 
    /// @param[in] aRhs コピー元の LotusCollection クラスへの参照。
    void LotusCollection::set(const LotusCollection& aRhs)
    {
        reset();
        
        for (int index = 0; index < aRhs.count(); ++index) {
            mLotuses[index] = aRhs[index];
        }
        
        mCount = aRhs.count();
    }

    //------------------------------------------------------------------------------
    /// 蓮のデータを初期化し、初期状態に戻します。
    /// 有効な蓮数は 0 となります。
    void LotusCollection::reset()
    {
        for (int index = 0; index < Parameter::LotusCountMax; ++index) {
            mLotuses[index].reset();
        }
        mCount = 0;
    }

    //------------------------------------------------------------------------------
    /// 蓮を追加します。
    ///
    void LotusCollection::setupAddLotus(const Vec2& aLotusPos, const float aRadius)
    {
        mLotuses[mCount++].reset(aLotusPos, aRadius);
    }

    //------------------------------------------------------------------------------
    /// @return 現在有効な蓮数
    int LotusCollection::count()const
    {
        return mCount;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex 有効な蓮のインデックス。
    ///
    /// @return aIndex 番目の有効な蓮への参照を返します。
    const Lotus& LotusCollection::operator[](int aIndex)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return mLotuses[aIndex];
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex 有効な蓮のインデックス。
    ///
    /// @return aIndex 番目の有効な蓮への参照を返します。
    Lotus& LotusCollection::operator[](int aIndex)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return mLotuses[aIndex];
    }
}

//------------------------------------------------------------------------------
// EOF
