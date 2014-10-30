//------------------------------------------------------------------------------
/// @file
/// @brief    HPCGame.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCGame.hpp"

#include "HPCCommon.hpp"
#include "HPCLevelDesigner.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    /// @param[in] aRandomSet 乱数クラス。
    Game::Game(RandomSet& aRandomSet)
        : mRandSet(aRandomSet)
        , mStage()
        , mCurrentStageIndex(0)
        , mRecord()
    {
    }

    //------------------------------------------------------------------------------
    /// 現在指定されているステージを開始します。
    ///
    /// @pre 現在のステージ番号が有効な範囲内にある必要があります。
    void Game::startStage()
    {
        HPC_ASSERT_MSG(isValidStage(), "Index indicates an invalid Stage (#%d)", mCurrentStageIndex);
        
        // ステージの生成を行います。
        LevelDesigner::Setup(mCurrentStageIndex, mStage, mRandSet.system());

        mStage.start();
        mRecord.writeStartStage(mCurrentStageIndex, mStage);
        mRecord.writeTurn(mStage.lastTurnResult());
    }

    //------------------------------------------------------------------------------
    /// 現ステージの1ターンをターン実行します。
    ///
    /// @pre 事前に startStage() が呼ばれ、現在のステージが実行中の状態である必要があります。
    void Game::runTurn()
    {
        HPC_ASSERT_MSG(isValidStage(), "Index indicates an invalid Stage (#%d)", mCurrentStageIndex);

        mStage.runTurn(mRandSet.game());
        mRecord.writeTurn(mStage.lastTurnResult());
    }

    //------------------------------------------------------------------------------
    /// 現ステージ内における進行状況を表す値を取得します。
    ///
    /// @return 現ステージ内における進行状況を表す StageStage 型の値。
    StageState Game::state()const
    {
        HPC_ASSERT_MSG(isValidStage(), "Index indicates an invalid Stage (#%d)", mCurrentStageIndex);
        return mStage.lastTurnResult().state;
    }

    //------------------------------------------------------------------------------
    /// ステージ終了時に、ステージが完了したことを通知するために呼び出します。
    ///
    /// @attention この関数を呼ぶと、ステージのインデックスが 1 追加されます。
    ///            そのため、最後のステージではステージ番号が無効になります。
    ///            ステージ開始前に、必ず isValidStage() 関数を利用してステージが有効かどうかを確認する必要があります。
    void Game::onStageDone()
    {
        HPC_ASSERT_MSG(isValidStage(), "Index indicates an invalid Stage (#%d)", mCurrentStageIndex);
        mRecord.writeEndStage(mStage);
        ++mCurrentStageIndex;
    }

    //------------------------------------------------------------------------------
    /// 内部で示されるステージ番号が有効な範囲を指しているかどうかを取得します。
    ///
    /// @note すべてのステージを終えた場合、内部の値は最終ステージのインデックスに 1 加えた値になります。
    ///       これを利用して、ステージがすべて終了したかどうかを取得することができます。
    ///
    /// @return ステージ番号が有効な範囲内にある場合は @c true を返し、
    ///         そうでない場合は @c false を返します。
    bool Game::isValidStage()const
    {
        return (0 <= mCurrentStageIndex && mCurrentStageIndex < Parameter::GameStageCount);
    }

    //------------------------------------------------------------------------------
    /// 内部に格納されているゲームの記録を返します。
    ///
    /// @return ゲームの記録を表す @c Record クラスへの const 参照を返します。
    const Record& Game::record()const
    {
        return mRecord;
    }
}

//------------------------------------------------------------------------------
// EOF
