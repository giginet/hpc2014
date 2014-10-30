//------------------------------------------------------------------------------
/// @file
/// @brief    HPCLevelDesigner.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCLevelDesigner.hpp"

#include <cstdlib>
#include "HPCCharaParam.hpp"
#include "HPCCollision.hpp"
#include "HPCCommon.hpp"
#include "HPCLevelGrid.hpp"
#include "HPCMath.hpp"
#include "HPCRandom.hpp"

namespace {
    using namespace hpc;

    //------------------------------------------------------------------------------
    /// 各ステージごとのグリッドサイズを取得します。
    ///
    /// 以下のルールに従い、各ステージ毎のグリッドサイズを返します。
    ///
    /// ステージ  |  横                               | 縦
    /// ----------------------------------------------|-------------------------------
    ///  0～ 9    |  10                               |  9 + 2ステージ毎に1ずつ増える
    /// 10～19    |  10 + 2ステージ毎に1ずつ増える    | 15
    /// 20～29    |  15 + 2ステージ毎に1ずつ増える    | 15 + 2ステージ毎に1ずつ増える
    /// それ以降  |  21                               | 21
    ///
    /// @param[in] aNumber レベル番号
    /// @return ステージのグリッドサイズを表す IntVec2
    IntVec2 GetStageGridSize(int aNumber)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aNumber, 0, Parameter::GameStageCount);
        const int stageDigit1 = aNumber % 10;
        if (0 <= aNumber && aNumber <= 9) {
            return IntVec2(10, 9 + stageDigit1 / 2 * 1);
        }
        else if (10 <= aNumber && aNumber <= 19) {
            return IntVec2(10 + stageDigit1 / 2 * 1, 15);
        }
        else if (20 <= aNumber && aNumber <= 29) {
            return IntVec2(15 + stageDigit1 / 2 * 1, 15 + stageDigit1 / 2 * 1);
        }
        else {
            return IntVec2(21, 21);
        }
    }

    //------------------------------------------------------------------------------
    /// フィールドのステージ生成用グリッドサイズを取得します。
    ///
    /// @return フィールドの1グリッドの大きさを返します。
    float FieldGridSize()
    {
        return Parameter::CharaRadius() * 2.0f;
    }

    //------------------------------------------------------------------------------
    /// フィールドの流れる速度を取得します。
    ///
    /// @param[in] aNumber レベル番号
    ///
    /// @return キャラに影響する、流れる速度を返します。
    Vec2 GetFieldFlowVel(int aNumber)
    {
        if (aNumber % 2 == 0) {
            // 偶数ステージは流れ無し
            return Vec2();
        } else {
            // 奇数ステージは、20ステージ毎に速度UP
            const float baseVel = 1.0f / 16.0f - 1.0f / 64.0f;
            const float unitVel = 1.0f / 64.0f;
            return Vec2(0.0f, baseVel + unitVel * (aNumber / 20));
        }
    }

    //------------------------------------------------------------------------------
    /// 対戦するキャラ数を取得します。
    ///
    /// @param[in] aNumber レベル番号
    ///
    /// @return プレイヤーを含めた、対戦するキャラ数を返します。
    int BattleCharaCount(int aNumber)
    {
        const int min = Parameter::CharaCountMin;
        const int max = Parameter::CharaCountMax;
        return min + static_cast<int>(
            (max - min) * Math::Min(static_cast<float>(aNumber) / 30.0f, 1.0f)
            );
    }

    //------------------------------------------------------------------------------
    /// グリッドを実数値の矩形に変換します。
    ///
    /// @param[in] aGridPos  矩形の左下を表すグリッドの位置を指定します。
    /// @param[in] aGridSize 矩形の大きさを指定します。
    ///
    /// @return グリッドから作成される実際の矩形を表す Rectangle
    Rectangle GridToRect(const IntVec2& aGridPos, const IntVec2& aGridSize)
    {
        const float left = FieldGridSize() * aGridPos.x;
        const float bottom = FieldGridSize() * aGridPos.y;
        const float width = FieldGridSize() * aGridSize.x;
        const float height = FieldGridSize() * aGridSize.y;

        return Rectangle(left, left + width, bottom, bottom + height);
    }

    //------------------------------------------------------------------------------
    /// ステージ端の配置禁止マージンを取得します。
    ///
    /// @return 配置禁止マージンを返します。
    int GetEdgeInhibitMargin()
    {
        return 1;
    }

    //------------------------------------------------------------------------------
    /// 各ステージごとのランダムに配置する蓮の個数を取得します。
    ///
    /// @param[in] aNumber ステージ番号
    ///
    /// @return ステージ aNumber で登場する蓮の個数
    int GetRandomLotusCount(int aNumber)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aNumber, 0, Parameter::GameStageCount);
        
        if (aNumber < 48) {
            const int num = aNumber;
            return Math::Min(num / 6 + 3, Parameter::LotusCountMax);
        } else {
            const int num = aNumber - 48;
            return Math::Min(num / 5 + 11, Parameter::LotusCountMax);
        }
    }

    //------------------------------------------------------------------------------
    /// @return 蓮のサイズ段階数
    int GetLotusSizeMax()
    {
        return 4;
    }

    //------------------------------------------------------------------------------
    /// 各ステージ毎のランダムな蓮の大きさを取得します。
    ///
    /// @param[in] aNumber  ステージ番号
    /// @param[in] aRand    乱数クラス
    ///
    /// @return 蓮の大きさ倍率を表す整数値
    int GetRandomLotusSize(int aNumber, Random& aRand)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aNumber, 0, Parameter::GameStageCount);
        // ステージ 0 ～ 4 はサイズ 2x のものだけ登場する
        if (aNumber <= 4) {
            static const int ItemTable_0_4[] = { 2 };
            const int index = aRand.randTerm(HPC_ARRAY_NUM(ItemTable_0_4));
            return ItemTable_0_4[index];
        }
        // ステージ 5 ～ 9 はサイズ 2x, 3x のものだけが 1 : 1 で登場する
        else if (aNumber <= 9) {
            static const int ItemTable_5_9[] = { 2, 3 };
            const int index = aRand.randTerm(HPC_ARRAY_NUM(ItemTable_5_9));
            return ItemTable_5_9[index];
        }
        // ステージ 10 ～ 24 はサイズ 2x, 3x, 4x のものが 2 : 2 : 1 で登場する
        else if (aNumber <= 24) {
            static const int ItemTable_10_24[] = { 2, 2, 3, 3, 4 };
            const int index = aRand.randTerm(HPC_ARRAY_NUM(ItemTable_10_24));
            return ItemTable_10_24[index];
        }
        // ステージ 25 ～ 49 はサイズ 1x, 2x, 3x, 4x のものが 1 : 2 : 3 : 2 で登場する
        else if (aNumber <= 49) {
            static const int ItemTable_25_49[] = { 1, 2, 2, 3, 3, 3, 4, 4 };
            const int index = aRand.randTerm(HPC_ARRAY_NUM(ItemTable_25_49));
            return ItemTable_25_49[index];
        }
        // ステージ 50 ～ 99 はサイズ 1x, 2x, 3x, 4x のものが 1 : 1 : 2 : 1 で登場する
        else {
            static const int ItemTable_50_99[] = { 1, 2, 3, 3, 4 };
            const int index = aRand.randTerm(HPC_ARRAY_NUM(ItemTable_50_99));
            return ItemTable_50_99[index];
        }
    }

    //------------------------------------------------------------------------------
    /// 各ステージごとのCPUの強さを取得します。
    ///
    /// @param[in] aNumber ステージ番号
    ///
    /// @return ステージ aNumber で登場するCPUの強さ
    int GetCpuStrength(int aNumber)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aNumber, 0, Parameter::GameStageCount);
        const float rate = static_cast<float>(aNumber + 1) / Parameter::GameStageCount;
        return Math::Min(static_cast<int>(rate * 100), 100);
    }

    //------------------------------------------------------------------------------
    /// グリッドの中央を座標に変換します。
    ///
    /// @param[in] aGridPos グリッドの左下位置を表す点。
    /// @param[in] aWidth   座標に変換したい矩形の aGridPos からの横幅 (右方向)。
    /// @param[in] aHeight  座標に変換したい矩形の aGridPos からの高さ (上方向)。
    ///
    /// @return 引数からつくられるグリッド四角形の中心を表す座標。
    Vec2 GridCenterToVec2(const IntVec2& aGridPos, int aWidth, int aHeight)
    {
        const Rectangle rect = GridToRect(aGridPos, IntVec2(aWidth, aHeight));
        return rect.center();
    }
}

namespace hpc {

    //------------------------------------------------------------------------------
    /// 渡された Stage に対しマップを生成します。
    ///
    /// @param[in]      aNumber ステージ番号
    /// @param[in,out]  aStage  ステージ情報。関数を呼ぶと書き換えられます。
    /// @param[in,out]  aRandom 乱数
    void LevelDesigner::Setup(int aNumber, Stage& aStage, Random& aRandom)
    {
        aStage.reset();

        HPC_RANGE_ASSERT_MIN_UB_I(aNumber, 0, Parameter::GameStageCount);
        // ステージの広さを決め、配置用グリッドを構成
        const IntVec2 stageGridSize = GetStageGridSize(aNumber);
        LevelGrid grid(stageGridSize, GetEdgeInhibitMargin(), aRandom);
        aStage.field().setup(
            GridToRect(IntVec2(), stageGridSize)
            , GetFieldFlowVel(aNumber)
            );
       
        // 蓮の個数を決め、グリッドに配置する
        aStage.lotuses().reset();
        const int lotusCount = GetRandomLotusCount(aNumber);
        HPC_RANGE_ASSERT_MIN_MAX_I(lotusCount, 1, Parameter::LotusCountMax);

        for (int index = 0; index < lotusCount; ++index) {
            // アイテムの大きさを決める。
            const int size = GetRandomLotusSize(aNumber, aRandom);
            HPC_RANGE_ASSERT_MIN_MAX_I(size, 1, GetLotusSizeMax());

            // 蓮の大きさはグリッドの大きさと一致しないため、グリッドをどれだけ占有するか計算する
            // 蓮の密集を防ぐために、占有グリッドを少し広げている
            const int occupyGridSize = Math::Ceil(Parameter::LotusBaseRadius() * 2 * size / FieldGridSize()) + 1;
            const IntVec2 gridPos = grid.setRandomOccupied(occupyGridSize, occupyGridSize);
            HPC_ASSERT(aStage.lotuses().count() <= lotusCount);
            aStage.lotuses().setupAddLotus(
                GridCenterToVec2(gridPos, occupyGridSize, occupyGridSize)
                , Parameter::LotusBaseRadius() * size
                );
        }

        // キャラの初期位置
        // 初期位置はキャラによって異なるが、最初の蓮までの距離は同じ
        {
            const int charaCount = BattleCharaCount(aNumber);
            
            Vec2 posArray[Parameter::CharaCountMax];
            
            // 配列に座標をセット
            {
                const Vec2 targetPos = aStage.lotuses()[0].pos();
                const Rectangle fieldRect = aStage.field().rect();
                const float largeSignX = targetPos.x - fieldRect.left < fieldRect.right - targetPos.x ? 1.0f : -1.0f;
                const float largeSignY = targetPos.y - fieldRect.bottom < fieldRect.top - targetPos.y ? 1.0f : -1.0f;
                const float rotSign = largeSignX * largeSignY;
                const float unitRotDeg = 90.0f / (Parameter::CharaCountMax);
                const int fieldHalfLenI = static_cast<int>(
                    Math::Min(fieldRect.width() / 2.0f, fieldRect.height() / 2.0f) - 1.0f
                    );
                HPC_ASSERT(0 < fieldHalfLenI);
                
                Vec2 baseVec(
                    // 最低3グリッド離れれば、初期配置で重なり合う事はない
                    aRandom.randMinMax(
                        Math::Max(
                            3 * static_cast<int>(FieldGridSize())
                            , fieldHalfLenI / 2
                            )
                        , fieldHalfLenI
                        ) * largeSignX
                    , 0.0f
                    );
                
                for (int index = 0; index < Parameter::CharaCountMax; ++index) {
                    posArray[index] = targetPos + baseVec;
                    baseVec.rotate(
                        Math::DegToRad(
                            float(unitRotDeg * rotSign)
                            )
                        );
                }
            }
            
            // 配列をシャッフル
            for (int index = 0; index < Parameter::CharaCountMax; ++index) {
                const int randIndex = aRandom.randMinTerm(index, Parameter::CharaCountMax);
                const Vec2 tmp = posArray[index];
                posArray[index] = posArray[randIndex];
                posArray[randIndex] = tmp;
            }
            
            // 初期配置で重なり合わない事を保証する
            {
                const float necessaryDist = Parameter::CharaRadius() * 2 + 0.125f;
                for (int indexA = 0; indexA < Parameter::CharaCountMax; ++indexA) {
                    for (int indexB = indexA + 1; indexB < Parameter::CharaCountMax; ++indexB) {
                        if (posArray[indexA].dist(posArray[indexB]) < necessaryDist) {
                            HPC_ASSERT_MSG(false, "CharaInitPos is overlapped.");
                        }
                    }
                }
            }
            
            const int cpuStrength = GetCpuStrength(aNumber);
            
            for (int index = 0; index < charaCount; ++index) {
                if (index == 0) {
                    // 0番は人間
                    aStage.charas().setupAddChara(
                        posArray[index]
                        , CharaParam::CreateHuman()
                        );
                } else {
                    // それ以外はCPU
                    aStage.charas().setupAddChara(
                        posArray[index]
                        , CharaParam::CreateCpu(
                            cpuStrength
                            )
                        );
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
// EOF
