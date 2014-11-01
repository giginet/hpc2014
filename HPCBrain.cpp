//------------------------------------------------------------------------------
/// @file
/// @brief    HPCBrain.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCBrain.hpp"

#include "HPCAnswer.hpp"
#include "HPCCollision.hpp"
#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCParameter.hpp"
#include "HPCRandom.hpp"
#include "HPCStageAccessor.hpp"

namespace {
    using namespace hpc;
    
    const int CpuSaveAccelTurnMax = 2;
}

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Brain::Brain()
        : mCharaParam()
        , mCpuSaveAccelTurn(0)
    {
        reset();
    }
    
    //------------------------------------------------------------------------------
    /// 状態をリセットします。
    void Brain::reset()
    {
        mCharaParam.reset();
    }
    
    //------------------------------------------------------------------------------
    /// 初期状態を設定します。
    ///
    /// @param[in] aCharaParam キャラのパラメータ。
    void Brain::setup(const CharaParam& aCharaParam)
    {
        mCharaParam = aCharaParam;
    }
    
    //------------------------------------------------------------------------------
    /// ステージ開始前の準備処理を行います。
    ///
    /// @param[in] aStageAccessor   ステージ情報へのアクセスを提供する
    ///                             StageAccessor クラスへの参照。
    void Brain::init(const StageAccessor& aStageAccessor)
    {
        switch (mCharaParam.type()) {
        case CharaType_Human:
            // Answer::Init でプレイヤーの初期状態を参照できるようにします。
            // 但し、Init でステージの状態を書き換えることはできません。
            Answer::Init(aStageAccessor);
            break;

        case CharaType_Cpu:
            initCpu(aStageAccessor);
            break;

        default:
            HPC_SHOULD_NOT_REACH_HERE();
        }
    }
    
    //------------------------------------------------------------------------------
    /// 次の動作を決定します。
    ///
    /// @param[in] aStageAccessor   ステージ情報へのアクセスを提供する
    ///                             StageAccessor クラスへの参照。
    /// @param[in] aRandom          乱数クラス。
    ///
    /// @return 次の動作
    Action Brain::getNextAction(
        const StageAccessor& aStageAccessor
        , Random& aRandom
        )
    {
        switch (mCharaParam.type()) {
        case CharaType_Human:
            return Answer::GetNextAction(aStageAccessor);

        case CharaType_Cpu:
            return getCpuNextAction(aStageAccessor, aRandom);

        default:
            HPC_SHOULD_NOT_REACH_HERE();
        }
        
        return Action();
    }
    
    //------------------------------------------------------------------------------
    /// CPUがステージ開始前の準備処理を行います。
    ///
    /// @param[in] aStageAccessor   ステージ情報へのアクセスを提供する
    ///                             StageAccessor クラスへの参照。
    void Brain::initCpu(const StageAccessor& aStageAccessor)
    {
        mCpuSaveAccelTurn = 0;
    }
    
    //------------------------------------------------------------------------------
    /// CPUが各種パラメータを基に、次の動作を決定します。
    ///
    /// @param[in] aStageAccessor   ステージ情報へのアクセスを提供する
    ///                             StageAccessor クラスへの参照。
    /// @param[in] aRandom          乱数クラス。
    ///
    /// @return 次の動作
    Action Brain::getCpuNextAction(
        const StageAccessor& aStageAccessor
        , Random& aRandom
        )
    {
        //return Action::Wait();
        const Chara& player = aStageAccessor.player();
        
        // 乱数を使うものは最初に計算
        const float slurDeg = static_cast<float>(
            aRandom.randMinMax(17, 20) * (aRandom.randTerm(2) == 0 ? -1 : 1)
            );
        const bool isSaveAccel = aRandom.randTerm(100) < 40;
        
        // 加速回数が0なら何もしない
        if (player.accelCount() == 0) {
            return Action::Wait();
        }
        
        // 加速節約フラグが立っていたら何もしない
        // ただし、一定ターン節約し続けていた場合は除く
        if (isSaveAccel) {
            if (mCpuSaveAccelTurn < CpuSaveAccelTurnMax) {
                ++mCpuSaveAccelTurn;
                return Action::Wait();
            }
        }
        mCpuSaveAccelTurn = 0;

        const int targetLotusNo = player.targetLotusNo();
        const Lotus& targetLotus = aStageAccessor.lotuses()[targetLotusNo];
        const Vec2 targetLotusPos = targetLotus.pos();
        Vec2 toTargetVec = targetLotusPos - player.pos();
        
        if (toTargetVec.isZero()) {
            return Action::Wait();
        }
        
        // 目標座標をぶれさせる
        toTargetVec.rotate(Math::DegToRad(slurDeg));
        
        return Action::Accel(player.pos() + toTargetVec);
    }
}
//------------------------------------------------------------------------------
// EOF
