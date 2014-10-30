//------------------------------------------------------------------------------
/// @file
/// @brief    HPCCharaCollection.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCCharaCollection.hpp"

#include "HPCCollision.hpp"
#include "HPCCommon.hpp"
#include "HPCStage.hpp"

namespace {
    using namespace hpc;
    
    //------------------------------------------------------------------------------
    /// 速度ベクトル計算用構造体
    struct CalcVelSet
    {
        Vec2 vels[Parameter::CharaCountMax];
        Vec2 ofsSeparateVec;
        int count;

        CalcVelSet()
            : vels()
            , ofsSeparateVec()
            , count(0)
        {
        }
        
        void addVel(const Vec2& aVel, const Vec2& aOfsSeparateVec)
        {
            HPC_ASSERT(count < Parameter::CharaCountMax);
            vels[count++] = aVel;
            ofsSeparateVec += aOfsSeparateVec;
        }
        
        Vec2 calculatedVel()const
        {
            if (count == 0) {
                return Vec2();
            }
            
            Vec2 totalVel;
            for (int index = 0; index < count; ++index) {
                totalVel += vels[index];
            }
            
            return totalVel / static_cast<float>(count);
        }
    };
    
    //------------------------------------------------------------------------------
    /// aCharaB より aCharaA が上位かどうかを返します。
    bool IsHighOrder(const Chara& aCharaA, const Chara& aCharaB)
    {
        // ゴールしている方が上位
        if (aCharaA.isGoal() && !aCharaB.isGoal()) {
            return true;
        }
        if (!aCharaA.isGoal() && aCharaB.isGoal()) {
            return false;
        }
        
        // 両方ゴールしている場合
        if (aCharaA.isGoal() && aCharaB.isGoal()) {
            // ゴールまでのターン数が短い方が上位
            return aCharaA.passedTurn() < aCharaB.passedTurn();
        }
        
        // 両方ゴールしていない場合
        if (!aCharaA.isGoal() && !aCharaB.isGoal()) {
            // 通過した蓮の数が多い方が上位
            return aCharaB.passedLotusCount() < aCharaA.passedLotusCount();
        }
        
        HPC_SHOULD_NOT_REACH_HERE();
        return false;
    }
}

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    CharaCollection::CharaCollection()
        : mCharas()
        , mCharaTypes()
        , mCount(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 各キャラの動作を決定します。
    void CharaCollection::procDecideAction(Random& aRandom)
    {
        for (int index = 0; index < count(); ++index) {
            Chara& chara = mCharas[index];
            
            // ゴールしていたら何もしない
            if (chara.isGoal()) {
                continue;
            }
            
            chara.decideAction(aRandom);
        }
    }

    //------------------------------------------------------------------------------
    /// 各キャラの動作を実行します。
    void CharaCollection::procExecAction()
    {
        for (int index = 0; index < count(); ++index) {
            Chara& chara = mCharas[index];
            
            // ゴールしていたら何もしない
            if (chara.isGoal()) {
                continue;
            }
            
            chara.execAction();
            
            // 移動処理を行う
            chara.move();
        }
    }

    //------------------------------------------------------------------------------
    /// 各キャラ同士の衝突判定を行います。
    void CharaCollection::procCheckColl()
    {
        // ■衝突判定の方針について
        // 条件：静止円同士での判定。非弾性衝突。処理順に影響しない。
        // 
        // 1. 各々の現在位置で衝突しているかをチェックする。
        // 2. 衝突している場合、現フレームの位置関係で衝突したと仮定し、互いの速度ベクトルを求める。
        // 3. 互いのめり込みを補正する為のベクトルを求める。
        // 4. 全てのキャラ同士で1. 2. 3. の判定を行い、求めた速度ベクトルの平均値を最終的な速度ベクトル、
        //    求めためり込み補正ベクトルの合計を、最終的な補正ベクトルとする。
        // 5. めり込み補正を行う。
        // 6. フィールド外に出ていたら、座標を内側に補正し、速度ベクトルをゼロにする。
        // (以上の処理を行った後、各種判定を行う)
        // 
        // ※注意点
        // 正確さよりもをシンプルさを優先している為、衝突の仕方によっては
        // 不自然な方向に跳ね返る事があります。
        
        CalcVelSet velSet[Parameter::CharaCountMax];
        
        for (int indexA = 0; indexA < count(); ++indexA) {
            const Chara& charaA = mCharas[indexA];
            
            // ゴールしていたら何もしない
            if (charaA.isGoal()) {
                continue;
            }
            
            const Vec2 velA = charaA.vel();
            const Circle circleA = charaA.region();
            
            for (int indexB = indexA + 1; indexB < count(); ++indexB) {
                const Chara& charaB = mCharas[indexB];
                
                // ゴールしていたら何もしない
                if (charaB.isGoal()) {
                    continue;
                }
                
                const Vec2 velB = charaB.vel();
                const Circle circleB = charaB.region();
                
                if (Collision::IsHit(circleA, circleB)) {
                    
                    Vec2 toB = charaB.pos() - charaA.pos();
                    const float margin = Parameter::CharaDecelSpeed();
                    const float separateHalfDist = (circleA.radius() + circleB.radius() - toB.length() + margin) / 2.0f;
                    // 完全に重なっていたら、x軸と水平に衝突したことにする
                    if (toB.isZero()) {
                        toB.x = 1.0f;
                    }
                    
                    const Vec2 verticalA = velA.getProjected(toB);
                    const Vec2 parallelA = velA - verticalA;
                    const Vec2 verticalB = velB.getProjected(toB);
                    const Vec2 parallelB = velB - verticalB;
                    
                    const float factor = Parameter::CharaReflectionFactor();
                    const Vec2 nextVerticalA = (verticalA * (1.0f - factor) + verticalB * (1.0f + factor)) / 2.0f;
                    const Vec2 nextVerticalB = nextVerticalA - (verticalB - verticalA) * factor;
                    
                    const Vec2 ofsSeparateVec = 0.0f < separateHalfDist
                        ? toB.getNormalized(separateHalfDist)
                        : Vec2();
                    velSet[indexA].addVel(parallelA + nextVerticalA, -ofsSeparateVec);
                    velSet[indexB].addVel(parallelB + nextVerticalB, ofsSeparateVec);
                }
            }
        }
        
        // 求めた結果を反映する
        for (int index = 0; index < count(); ++index) {
            Chara& chara = mCharas[index];
            
            // ゴールしていたら何もしない
            if (chara.isGoal()) {
                continue;
            }
            
            // 衝突していなかったら何もしない
            if (velSet[index].count == 0) {
                continue;
            }
            chara.setVel(velSet[index].calculatedVel());
            
            // めりこみ補正を反映させる
            chara.separation(velSet[index].ofsSeparateVec);
        }
        
        // フィールド外に出ていたら、内側に補正する
        for (int index = 0; index < count(); ++index) {
            Chara& chara = mCharas[index];
            
            // ゴールしていたら何もしない
            if (chara.isGoal()) {
                continue;
            }
            
            chara.correctInside();
        }
    }

    //------------------------------------------------------------------------------
    /// 最終処理を行います。
    void CharaCollection::procEnd(const Stage& aStage)
    {
        for (int index = 0; index < count(); ++index) {
            Chara& chara = mCharas[index];
            
            // ゴールしていたら何もしない
            if (chara.isGoal()) {
                continue;
            }
            
            // ターン経過処理を行う
            chara.updateTurn();
            
            // 蓮の通過判定
            while (!chara.isGoal()) {
                const Lotus& lotus = aStage.lotuses()[chara.targetLotusNo()];
                // 円（蓮）と移動円（キャラの前回位置から今回位置への移動）で衝突判定を行う
                if (Collision::IsHit(lotus.region(), chara.prevRegion(), chara.region().pos())) {
                    // 目標の蓮を通過したら、次の蓮との判定を行う
                    chara.incTargetLotusNo();
                } else {
                    // 目標の蓮を通過していなかったら判定終了
                    break;
                }
            }
        }
        
        // 順位を更新
        updateRank();
    }

    //------------------------------------------------------------------------------
    /// キャラのデータを初期化し、初期状態に戻します。
    /// 有効なキャラ数は 0 となります。
    void CharaCollection::reset()
    {
        for (int index = 0; index < Parameter::CharaCountMax; ++index) {
            mCharas[index].reset();
            mCharaTypes[index] = CharaType_TERM;
        }
        mCount = 0;
    }

    //------------------------------------------------------------------------------
    /// キャラをコレクションの末尾に追加します。
    ///
    void CharaCollection::setupAddChara(
        const Vec2& aCharaPos
        , const CharaParam& aCharaParam
        )
    {
        HPC_RANGE_ASSERT_MIN_UB_I(mCount, 0, Parameter::CharaCountMax);
        mCharas[mCount].setup(aCharaPos, aCharaParam);
        mCharaTypes[mCount] = aCharaParam.type();
        ++mCount;
    }

    //------------------------------------------------------------------------------
    /// @return 現在有効なキャラ数。
    int CharaCollection::count()const
    {
        return mCount;
    }

    //------------------------------------------------------------------------------
    /// @return 人間キャラが全員ゴールしたか。
    bool CharaCollection::isAllHumanGoal()const
    {
        int targetCount = 0;
        
        for (int index = 0; index < count(); ++index) {
            if (mCharaTypes[index] != CharaType_Human) {
                continue;
            }
            
            if (!mCharas[index].isGoal()) {
                return false;
            }
            
            ++targetCount;
        }
        
        // 人間キャラが存在しないならfalseを返す
        if (targetCount == 0) {
            return false;
        }
        
        return true;
    }

    //------------------------------------------------------------------------------
    /// @return ゴールしたキャラ数。
    int CharaCollection::goalCount()const
    {
        int targetCount = 0;
        
        for (int index = 0; index < count(); ++index) {
            if (mCharas[index].isGoal()) {
                ++targetCount;
            }
        }
        
        return targetCount;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex キャラのインデックス。
    ///
    /// @return aIndex 番目のキャラへの参照を返します。
    const Chara& CharaCollection::operator[](int aIndex)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return mCharas[aIndex];
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex キャラのインデックス。
    ///
    /// @return aIndex 番目のキャラへの参照を返します。
    Chara& CharaCollection::operator[](int aIndex)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return mCharas[aIndex];
    }

    //------------------------------------------------------------------------------
    /// 順位を更新します。
    void CharaCollection::updateRank()
    {
        Chara* charaArray[Parameter::CharaCountMax] = {0};
        
        for (int index = 0; index < count(); ++index) {
            charaArray[index] = &mCharas[index];
        }
        
        // 順位決定
        for (int indexA = 0; indexA < count(); ++indexA) {
            HPC_ASSERT(charaArray[indexA] != 0);
            for (int indexB = indexA + 1; indexB < count(); ++indexB) {
                HPC_ASSERT(charaArray[indexB] != 0);
                if (IsHighOrder(*charaArray[indexB], *charaArray[indexA])) {
                    Chara* tmp = charaArray[indexA];
                    charaArray[indexA] = charaArray[indexB];
                    charaArray[indexB] = tmp;
                }
            }
        }
        
        // 結果を反映
        for (int index = 0; index < count(); ++index) {
            charaArray[index]->setRank(index);
        }
    }
}

//------------------------------------------------------------------------------
// EOF
