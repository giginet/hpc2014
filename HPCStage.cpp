//------------------------------------------------------------------------------
/// @file
/// @brief    HPCStage.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCStage.hpp"

#include "HPCCommon.hpp"
#include "HPCLevelDesigner.hpp"
#include "HPCParameter.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Stage::Stage()
        : mCharas()
        , mLotuses()
        , mField()
        , mTurnResult()
        , mTurnIndex(0)
    {
    }

    //------------------------------------------------------------------------------
    /// ステージ情報をすべて削除します。
    void Stage::reset()
    {
        mCharas.reset();
        mLotuses.reset();
        mField.reset();
        mTurnResult.reset();
        mTurnIndex = 0;
    }

    //------------------------------------------------------------------------------
    /// ステージ開始時に一度だけ呼ぶことで、ステージの初期化処理を行います。
    ///
    /// @note 関数を呼んだ後は、内部で保持している TurnResult に初期状態が記録されます。
    ///       そのため、ステージ開始後すぐに lastTurnResult() を呼ぶことで、
    ///       ステージの初期状態を取得することができます。
    void Stage::start()
    {
        mTurnResult.state = StageState_Playing;
        mTurnIndex = 0;

        // Stage情報を基に、各キャラが準備処理を行います。
        for (int index = 0; index < mCharas.count(); ++index) {
            mCharas[index].init(*this, index);
        }
        
        updateTurnResult();
    }

    //------------------------------------------------------------------------------
    /// Stage クラスに内部的に保持されているターンの実行結果を返します。
    ///
    /// @return 内部で保持されている TurnResult の値。
    const TurnResult& Stage::lastTurnResult()const
    {
        return mTurnResult;
    }

    //------------------------------------------------------------------------------
    /// ターンを1つ進める処理を行います。
    /// 各キャラの動作(Chara::act)の結果に従い、
    /// 実際のステージの状態を変化させます。
    void Stage::runTurn(Random& aRandom)
    {
        HPC_ASSERT(mTurnResult.state == StageState_Playing);
        mTurnResult.reset();
        
        // 各キャラの動作を確定する
        mCharas.procDecideAction(aRandom);
        
        // 動作が確定したら、動作を実行する
        mCharas.procExecAction();
        
        // 動作が実行されたら、キャラ同士の衝突判定を行う
        mCharas.procCheckColl();
        
        // 衝突判定が終わったら、最終処理を行う
        mCharas.procEnd(*this);
        
        // 結果の保存
        updateTurnResult();
        
        mTurnResult.state = StageState_Playing;
        
        // 人間キャラが全員ゴールしたらゲームクリア
        if (mCharas.isAllHumanGoal()) {
            mTurnResult.state = StageState_Complete;
            return;
        }
        
        // 残り１人になったらゲーム終了
        if (mCharas.goalCount() == mCharas.count() - 1) {
            mTurnResult.state = StageState_Failed;
            return;
        }

        // ターン数が一定数を超えたら終了
        if (++mTurnIndex >= Parameter::GameTurnPerStage) {
            mTurnResult.state = StageState_TurnLimit;
            return;
        }
    }

    //------------------------------------------------------------------------------
    CharaCollection& Stage::charas()
    {
        return mCharas;
    }

    //------------------------------------------------------------------------------
    const CharaCollection& Stage::charas()const
    {
        return mCharas;
    }

    //------------------------------------------------------------------------------
    LotusCollection& Stage::lotuses()
    {
        return mLotuses;
    }

    //------------------------------------------------------------------------------
    const LotusCollection& Stage::lotuses()const
    {
        return mLotuses;
    }

    //------------------------------------------------------------------------------
    const Field& Stage::field()const
    {
        return mField;
    }

    //------------------------------------------------------------------------------
    Field& Stage::field()
    {
        return mField;
    }

    //------------------------------------------------------------------------------
    /// TurnResultの情報を更新します。
    /// キャラの情報を更新します。
    void Stage::updateTurnResult()
    {
        for (int index = 0; index < mCharas.count(); ++index) {
            mTurnResult.charas[index].pos = mCharas[index].pos();
            mTurnResult.charas[index].accelCount = mCharas[index].accelCount();
            mTurnResult.charas[index].passedLotusCount = mCharas[index].passedLotusCount();
        }
    }
}

//------------------------------------------------------------------------------
// EOF
