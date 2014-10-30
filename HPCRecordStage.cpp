//------------------------------------------------------------------------------
/// @file
/// @brief    HPCRecordStage.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCRecordStage.hpp"

#include "HPCCommon.hpp"
#include "HPCLevelDesigner.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    RecordStage::RecordStage()
        : mCurrentTurn(0)
        , mRanks()
        , mPassedLotusCount(0)
        , mCharaCount(0)
#ifdef DEBUG
        , mTurns()
        , mField()
        , mLotuses()
        , mInitPositions()
#endif
        , mIsFailed(false)
    {
    }

    //------------------------------------------------------------------------------
    /// ステージの記録を開始することを通知します。
    ///
    /// @param[in] aStage 現在実行しているステージを表す Stage クラスへの参照。
    void RecordStage::writeStart(const Stage& aStage)
    {
        mCharaCount = aStage.charas().count();
        
#ifdef DEBUG
        mField.set(aStage.field());
        mLotuses.set(aStage.lotuses());
        for (int index = 0; index < mCharaCount; ++index) {
            mInitPositions[index] = aStage.charas()[index].pos();
        }
#endif
    }

    //------------------------------------------------------------------------------
    /// 毎ターンの記録を行います。
    ///
    /// @param[in] aResult 現在のターンを表す実行結果。
    void RecordStage::writeTurn(const TurnResult& aResult)
    {
#ifdef DEBUG
        HPC_RANGE_ASSERT_MIN_UB_I(mCurrentTurn, 0, HPC_ARRAY_NUM(mTurns));
        mTurns[mCurrentTurn].set(aResult);
#endif
        ++mCurrentTurn;
        // 得点計算のため、失敗したことを記録しておく。
        if (
            aResult.state == StageState_Failed
            || aResult.state == StageState_TurnLimit
        ) {
            mIsFailed = true;
        }
    }

    //------------------------------------------------------------------------------
    /// 終了時の記録を行います。
    ///
    /// @param[in] aStage 現在実行しているステージを表す Stage クラスへの参照。
    void RecordStage::writeEnd(const Stage& aStage)
    {
        for (int index = 0; index < mCharaCount; ++index) {
            mRanks[index] = aStage.charas()[index].rank();
        }
        
        // 通過した蓮の数を計算
        const Chara& player = aStage.charas()[0];
        mPassedLotusCount = player.passedLotusCount();
    }

    //------------------------------------------------------------------------------
    /// @return ステージ毎の得点。誤差を防止するため、 double で返します。
    double RecordStage::score()const
    {
        // ステージごとの点数は、
        // 通過蓮スコア ＝ (通過した蓮の数 ÷ 必要周回数)^2 とすると、
        // 通過蓮スコア × (通過蓮スコア / クリアに掛かったターン数) × 一定の係数
        // × 順位による倍率
        // で求める。
        static const int Value = 250;
        static const double RankRateTable[Parameter::CharaCountMax] = {
            6.0
            , 4.0
            , 2.0
            , 1.0
            };
        const int turn = mIsFailed ? Parameter::GameTurnPerStage : (mCurrentTurn + 1);
        const int passedLotusScore =
            (mPassedLotusCount / Parameter::StageRoundCount)
            * (mPassedLotusCount / Parameter::StageRoundCount);
        
        double totalScore = passedLotusScore * passedLotusScore / static_cast<double>(turn) * Value;
        // クリアしていたら順位ボーナスを乗算
        if (!mIsFailed) {
            // 倍率テーブルのインデックス。対戦人数が少なければ、その分後ろにずらす。
            const int rateTableIndex = mRanks[0] + (Parameter::CharaCountMax - mCharaCount);
            HPC_ASSERT(0 <= rateTableIndex && rateTableIndex < Parameter::CharaCountMax);
            totalScore *= RankRateTable[rateTableIndex];
        }
        return totalScore;
    }

    //------------------------------------------------------------------------------
    /// 記録された結果を画面に出力します。
    void RecordStage::dump()const
    {
#ifdef DEBUG
        HPC_PRINT_LOG(
            "Field", "(%7.2f,%7.2f)-(%7.2f,%7.2f)\n"
            , mField.rect().left
            , mField.rect().bottom
            , mField.rect().right
            , mField.rect().top
            );
        for (int index = 0; index < mLotuses.count(); ++index) {
            const Circle& lotusRegion = mLotuses[index].region();
            HPC_PRINT_LOG("Lotus", "#%3d: (%7.2f,%7.2f) R=%7.2f\n", 
                index, lotusRegion.pos().x, lotusRegion.pos().y, lotusRegion.radius());
        }
        for (int index = 0; index < mCurrentTurn; ++index) {
            const TurnResult& turn = mTurns[index];
            HPC_PRINT_LOG("Turn", "#%04d: ", index);
            switch(turn.state) {
            case StageState_Playing:
                HPC_PRINT(" ");
                break;

            case StageState_Failed:
                HPC_PRINT("F");
                break;

            case StageState_Complete:
                HPC_PRINT("C");
                break;

            case StageState_TurnLimit:
                HPC_PRINT("L");
                break;

            default:
                HPC_SHOULD_NOT_REACH_HERE();
                break;
            }
            HPC_PRINT("\n");
            for (int charaIndex = 0; charaIndex < mCharaCount; ++charaIndex) {
                HPC_PRINT(
                    " chara[%d] - [%7.2f,%7.2f]"
                    , charaIndex
                    , turn.charas[charaIndex].pos.x
                    , turn.charas[charaIndex].pos.y
                    );
                HPC_PRINT("\n");
            }
        }
#endif
        HPC_PRINT_LOG("Score", "%d\n", static_cast<int>(score()));
    }

    //------------------------------------------------------------------------------
    /// 記録された結果をJSON形式で出力します。
    ///
    /// @param[in] isCompressed 圧縮された状態で出力するかどうか。
    ///                         @c true にすると、改行やインデントを除いた形で出力されます。
    void RecordStage::dumpJson(bool isCompressed)const
    {
#ifdef DEBUG
        HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
        HPC_PRINT("[");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            // 初期状態情報
            HPC_PRINT_JSON_DEBUG(!isCompressed, "            "); // インデント (12)
            HPC_PRINT("[");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // フィールド情報
                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("%7.3f,%7.3f,", mField.rect().width(), mField.rect().height());
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // 蓮情報
                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("[");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                for (int lotusIndex = 0; lotusIndex < mLotuses.count(); ++lotusIndex) {
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "                    "); // インデント (20)
                    HPC_PRINT("[");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                        HPC_PRINT_JSON_DEBUG(!isCompressed, "                        "); // インデント (24)
                        HPC_PRINT(
                            "%7.3f,%7.3f,%7.3f"
                            , mLotuses[lotusIndex].pos().x
                            , mLotuses[lotusIndex].pos().y
                            , mLotuses[lotusIndex].radius()
                            );
                        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "                    "); // インデント (20)
                    HPC_PRINT("]");
                    if (lotusIndex + 1 < mLotuses.count()) {
                        HPC_PRINT(",");
                    }
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                }

                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("],");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // 順位情報
                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("[");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                    HPC_PRINT_JSON_DEBUG(!isCompressed, "                    "); // インデント (20)
                    for (int charaIndex = 0; charaIndex < mCharaCount; ++charaIndex) {
                        HPC_PRINT("%d", mRanks[charaIndex]);
                        if (charaIndex < mCharaCount - 1) {
                            HPC_PRINT(", ");
                        }
                    }
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("],");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // 流れる速度
                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("%7.6f,", mField.flowVel().y);
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // スコア
                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("%d", static_cast<int>(score()));
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            HPC_PRINT_JSON_DEBUG(!isCompressed, "            "); // インデント (12)
            HPC_PRINT("],");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            // ターン情報
            HPC_PRINT_JSON_DEBUG(!isCompressed, "            "); // インデント (12)
            HPC_PRINT("[");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            for (int turn = 0; turn < mCurrentTurn; ++turn) {
                const TurnResult& s = mTurns[turn];
                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("[");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                    // キャラ情報
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "                    "); // インデント (20)
                    HPC_PRINT("[");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                    for (int charaIndex = 0; charaIndex < mCharaCount; ++charaIndex) {
                        HPC_PRINT_JSON_DEBUG(!isCompressed, "                        "); // インデント (24)
                        HPC_PRINT("[");
                        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                            HPC_PRINT_JSON_DEBUG(!isCompressed, "                            "); // インデント (28)
                            HPC_PRINT(
                                "%7.3f,%7.3f,"
                                , s.charas[charaIndex].pos.x
                                , s.charas[charaIndex].pos.y
                                );
                            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                            HPC_PRINT_JSON_DEBUG(!isCompressed, "                            "); // インデント (28)
                            HPC_PRINT(
                                "%d,"
                                , s.charas[charaIndex].accelCount
                                );
                            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                            HPC_PRINT_JSON_DEBUG(!isCompressed, "                            "); // インデント (28)
                            HPC_PRINT(
                                "%d"
                                , s.charas[charaIndex].passedLotusCount
                                );
                            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                        HPC_PRINT_JSON_DEBUG(!isCompressed, "                        "); // インデント (24)
                        HPC_PRINT("]");
                        if (charaIndex + 1 < mCharaCount) {
                            HPC_PRINT(",");
                        }
                        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                    }

                    HPC_PRINT_JSON_DEBUG(!isCompressed, "                    "); // インデント (20)
                    HPC_PRINT("]");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                HPC_PRINT_JSON_DEBUG(!isCompressed, "                "); // インデント (16)
                HPC_PRINT("]");
                if (turn + 1 < mCurrentTurn) {
                    HPC_PRINT(",");
                }
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
            }

            HPC_PRINT_JSON_DEBUG(!isCompressed, "            "); // インデント (12)
            HPC_PRINT("]");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
        HPC_PRINT("]");
#else
        // デバッグ無効の場合、json 出力はサポートされません。
        HPC_PRINT("[]");
#endif
    }
}

//------------------------------------------------------------------------------
// EOF
