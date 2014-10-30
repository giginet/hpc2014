//------------------------------------------------------------------------------
/// @file
/// @brief    HPCStageAccessor.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCStageAccessor.hpp"

#include "HPCChara.hpp"
#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCParameter.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    StageAccessor::StageAccessor()
        : mStagePtr(0)
        , mEnemyAccessor()
        , mPlayerIndex(-1)
    {
    }

    //------------------------------------------------------------------------------
    /// 初期設定を行います。
    ///
    /// @param[in] aStage       現在実行しているステージを表す Stage クラスへの参照。
    /// @param[in] aPlayerIndex プレイヤーのインデックス。
    void StageAccessor::init(const Stage& aStage, int aPlayerIndex)
    {
        mStagePtr = &aStage;
        mEnemyAccessor.init(aStage.charas(), aPlayerIndex);
        mPlayerIndex = aPlayerIndex;
    }

    //------------------------------------------------------------------------------
    /// プレイヤーの参照を返します。
    /// 参加者にとってのプレイヤーは参加者自身であり、
    /// CPUにとってのプレイヤーはCPU自身です。
    /// 
    /// @return プレイヤー情報。
    const Chara& StageAccessor::player()const
    {
        return mStagePtr->charas()[mPlayerIndex];
    }

    //------------------------------------------------------------------------------
    /// @return 敵キャラ情報。
    const EnemyAccessor& StageAccessor::enemies()const
    {
        return mEnemyAccessor;
    }

    //------------------------------------------------------------------------------
    /// @return 蓮情報。
    const LotusCollection& StageAccessor::lotuses()const
    {
        return mStagePtr->lotuses();
    }

    //------------------------------------------------------------------------------
    /// @return フィールド情報。
    const Field& StageAccessor::field()const
    {
        return mStagePtr->field();
    }
}
//------------------------------------------------------------------------------
// EOF
