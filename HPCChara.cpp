//------------------------------------------------------------------------------
/// @file
/// @brief    HPCChara.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCChara.hpp"

#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCParameter.hpp"
#include "HPCRandom.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Chara::Chara()
        : mStageAccessor()
        , mBrain()
        , mRegion(Vec2(), Parameter::CharaRadius())
        , mPrevRegion(Vec2(), Parameter::CharaRadius())
        , mDecidedAction()
        , mVel()
        , mAccelCount(0)
        , mAccelWaitTurn(0)
        , mAccelWaitTurnMax(0)
        , mTargetLotusNo(0)
        , mRoundCount(0)
        , mRank(0)
        , mPassedTurn(0)
    {
        reset();
    }

    //------------------------------------------------------------------------------
    /// ステージ開始前の準備処理を行います。
    ///
    /// @param[in] aStage   現在実行しているステージを表す Stage クラスへの参照。
    void Chara::init(const Stage& aStage, int aCharaIndex)
    {
        mStageAccessor.init(aStage, aCharaIndex);
        mBrain.init(mStageAccessor);
    }

    //------------------------------------------------------------------------------
    /// 動作を決定します。
    void Chara::decideAction(Random& aRandom)
    {
        // 衝突判定用に、前回領域を覚えておく
        mPrevRegion = mRegion;
        
        mDecidedAction = mBrain.getNextAction(mStageAccessor, aRandom);
    }

    //------------------------------------------------------------------------------
    /// 動作を実行します。
    void Chara::execAction()
    {
        switch (mDecidedAction.type()) {
        case ActionType_Wait:
            // 何もしない
            break;

        case ActionType_Accel:
            // 加速できるなら加速
            accelIfPossible(mDecidedAction.value());
            break;

        default:
            HPC_SHOULD_NOT_REACH_HERE();
            break;
        }
        
        // 実行済みActionをリセットしておく
        mDecidedAction.reset();
    }

    //------------------------------------------------------------------------------
    /// 移動処理を行います。
    void Chara::move()
    {
        // 速度分移動 ＆ フィールドの流れる速度を反映
        mRegion.move(mVel + mStageAccessor.field().flowVel());
        
        // 減速させる
        if (!mVel.isZero()) {
            const float len = Math::Max(
                mVel.length() - Parameter::CharaDecelSpeed()
                , 0.0f
                );
            if (0.0f < len) {
                mVel.normalize(len);
            } else {
                mVel.reset();
            }
        }
    }

    //------------------------------------------------------------------------------
    /// めり込み補正を行います。
    void Chara::separation(const Vec2& aSeparateVec)
    {
        mRegion.move(aSeparateVec);
    }

    //------------------------------------------------------------------------------
    /// ターン経過処理を行います。
    void Chara::updateTurn()
    {
        HPC_ASSERT(!isGoal());
        
        ++mPassedTurn;
        --mAccelWaitTurn;
        if (mAccelWaitTurn <= 0) {
            mAccelCount = Math::Min(mAccelCount + 1, Parameter::CharaAccelCountMax);
            mAccelWaitTurn = mAccelWaitTurnMax;
        }
    }

    //------------------------------------------------------------------------------
    /// フィールドの内側に補正します。
    /// フィールド外に出ていた場合、座標補正と同時に速度がゼロになります。
    void Chara::correctInside()
    {
        const Rectangle fieldRect = mStageAccessor.field().rect();
        
        Vec2 myPos = mRegion.pos();
        const float radius = mRegion.radius();
        bool isCorrect = false;
        
        if (myPos.x - radius < fieldRect.left) {
            myPos.x = fieldRect.left + radius;
            isCorrect = true;
        } else if (fieldRect.right < myPos.x + radius) {
            myPos.x = fieldRect.right - radius;
            isCorrect = true;
        }
        if (myPos.y - radius < fieldRect.bottom) {
            myPos.y = fieldRect.bottom + radius;
            isCorrect = true;
        } else if (fieldRect.top < myPos.y + radius) {
            myPos.y = fieldRect.top - radius;
            isCorrect = true;
        }
        
        if (isCorrect) {
            mRegion.setPos(myPos);
            mVel.reset();
        }
    }

    //------------------------------------------------------------------------------
    /// 状態をリセットします。
    void Chara::reset()
    {
        mRegion.setPos(hpc::Vec2());
        mPrevRegion.setPos(hpc::Vec2());
        mBrain.reset();
        mVel.reset();
        mAccelCount = Parameter::CharaInitAccelCount;
        mAccelWaitTurn = Parameter::CharaAddAccelWaitTurn;
        mAccelWaitTurnMax = Parameter::CharaAddAccelWaitTurn;
        mTargetLotusNo = 0;
        mRoundCount = 0;
        mRank = 0;
        mPassedTurn = 0;
    }

    //------------------------------------------------------------------------------
    /// 初期状態を設定します。
    ///
    /// @param[in] aPos       初期位置
    /// @param[in] aCharaType キャラのパラメータ
    void Chara::setup(const Vec2& aPos, const CharaParam& aCharaParam)
    {
        mRegion.setPos(aPos);
        mPrevRegion.setPos(aPos);
        mBrain.setup(aCharaParam);
        
        // パラメータ設定
        {
            const float strRate = static_cast<float>(aCharaParam.strength()) / 100.0f;
            mAccelWaitTurnMax = Parameter::CharaAddAccelWaitTurn - static_cast<int>(strRate * 2 + 0.5f);
        }
    }

    //------------------------------------------------------------------------------
    /// 次に目指す蓮の番号を１つ進めます。
    void Chara::incTargetLotusNo()
    {
        ++mTargetLotusNo;
        // 一周回ったら周回数加算
        if (mStageAccessor.lotuses().count() == mTargetLotusNo) {
            mTargetLotusNo = 0;
            ++mRoundCount;
        }
    }

    //------------------------------------------------------------------------------
    /// 速度を設定します。
    ///
    /// @param[in] aVel       速度
    void Chara::setVel(const Vec2& aVel)
    {
        mVel = aVel;
    }

    //------------------------------------------------------------------------------
    /// 順位を設定します。
    ///
    /// @param[in] aRank    順位
    void Chara::setRank(const int aRank)
    {
        mRank = aRank;
    }

    //------------------------------------------------------------------------------
    /// キャラの領域を表す円を返します。
    ///
    /// @return キャラの領域を表す Circle
    const Circle& Chara::region()const
    {
        return mRegion;
    }

    //------------------------------------------------------------------------------
    /// @return キャラの現在位置
    Vec2 Chara::pos()const
    {
        return mRegion.pos();
    }

    //------------------------------------------------------------------------------
    /// @return キャラの現在速度
    Vec2 Chara::vel()const
    {
        return mVel;
    }

    //------------------------------------------------------------------------------
    /// @return ゴールしたか
    bool Chara::isGoal()const
    {
        return mRoundCount == Parameter::StageRoundCount;
    }

    //------------------------------------------------------------------------------
    /// @return 加速できる回数
    int Chara::accelCount()const
    {
        return mAccelCount;
    }

    //------------------------------------------------------------------------------
    /// @return 加速回数が増えるまでの残りターン数
    int Chara::accelWaitTurn()const
    {
        return mAccelWaitTurn;
    }

    //------------------------------------------------------------------------------
    /// @return 現在の目指す蓮番号
    int Chara::targetLotusNo()const
    {
        return mTargetLotusNo;
    }

    //------------------------------------------------------------------------------
    /// @return 周回数
    int Chara::roundCount()const
    {
        return mRoundCount;
    }

    //------------------------------------------------------------------------------
    /// @return 順位
    int Chara::rank()const
    {
        return mRank;
    }

    //------------------------------------------------------------------------------
    /// @return 通過した蓮の数
    int Chara::passedLotusCount()const
    {
        return mRoundCount * mStageAccessor.lotuses().count() + mTargetLotusNo;
    }

    //------------------------------------------------------------------------------
    /// @return 経過ターン数
    int Chara::passedTurn()const
    {
        return mPassedTurn;
    }

    //------------------------------------------------------------------------------
    /// キャラの前回領域を表す円を返します。
    ///
    /// @return キャラの領域を表す Circle
    const Circle& Chara::prevRegion()const
    {
        return mPrevRegion;
    }

    //------------------------------------------------------------------------------
    /// 加速できるなら加速します。
    void Chara::accelIfPossible(const Vec2& aTargetPos)
    {
        // 加速可能回数がゼロの場合、何もしない
        if (mAccelCount <= 0) {
            return;
        }
        
        const Vec2 toTargetVec = aTargetPos - pos();
        
        // 目標座標とキャラ座標が同値の場合、何もしない
        if (toTargetVec.isZero()) {
            return;
        }
        
        --mAccelCount;
        
        // 目標座標方向への一定加速度を設定する（加算ではなく、上書き）
        mVel = toTargetVec.getNormalized(Parameter::CharaAccelSpeed());
    }
}
//------------------------------------------------------------------------------
// EOF
