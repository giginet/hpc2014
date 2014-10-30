//------------------------------------------------------------------------------
/// @file
/// @brief    HPCRecord.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCRecord.hpp"

#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Record::Record()
        : mStage()
        , mCurrentStageIndex(0)
    {
    }

    //------------------------------------------------------------------------------
    /// ステージ開始時に一度呼ぶことで、ステージ開始を記録します。
    ///
    /// @param[in] aStageIndex  ステージ番号。
    /// @param[in] aStage       ステージ情報への参照。
    ///
    /// @pre ステージ番号は有効な範囲を示している必要があります。
    void Record::writeStartStage(int aStageIndex, const Stage& aStage)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aStageIndex, 0, Parameter::GameStageCount);

        mCurrentStageIndex = aStageIndex;
        mStage[mCurrentStageIndex].writeStart(aStage);
    }
    
    //------------------------------------------------------------------------------
    /// ステージの各ターンの状態を記録します。
    ///
    /// @pre 記録前に writeStartStage を呼び、ステージ開始状態にする必要があります。
    ///
    /// @param[in] aResult ターンの実行結果。
    void Record::writeTurn(const TurnResult& aResult)
    {
        mStage[mCurrentStageIndex].writeTurn(aResult);
    }

    //------------------------------------------------------------------------------
    /// ステージ終了時に一度呼ぶことで、終了時の結果を記録します。
    ///
    /// @param[in] aStage       ステージ情報への参照。
    ///
    /// @pre ステージ番号は有効な範囲を示している必要があります。
    void Record::writeEndStage(const Stage& aStage)
    {
        mStage[mCurrentStageIndex].writeEnd(aStage);
    }

    //------------------------------------------------------------------------------
    /// 各ステージの合計得点を返します。
    /// すべてのステージが終了してから呼びます。
    /// 記録中に呼び出した場合、正常な結果を得ることが出来ません。
    ///
    /// @return 各ステージの合計得点。
    int Record::score()const
    {
        // スコアの誤差が出ないように 浮動小数で計算し、
        // 最後に整数に丸める。
        //
        // dobule を使うのは、float の加算を行って値が大きくなる可能性があるため。
        double total = 0;
        for (int index = 0; index < Parameter::GameStageCount; ++index) {
            total += mStage[index].score();
        }
        return static_cast<int>(total);
    }

    //------------------------------------------------------------------------------
    /// 引数に指定されたステージの記録を一覧形式で画面に表示します。
    /// 記録がされていないステージの番号を指定した場合は何も表示されません。
    ///
    /// @param[in] aStageIndex ステージ番号。有効な範囲の番号が指定される必要があります。
    void Record::dumpStage(int aStageIndex)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aStageIndex, 0, Parameter::GameStageCount);
        
        HPC_PRINT_LOG("Stage", "%d\n", aStageIndex);
        mStage[aStageIndex].dump();
    }


    //------------------------------------------------------------------------------
    /// 引数に指定されたステージの記録を JSON 形式で画面に出力します。
    /// 画面には指定されたステージの記録に関する部分のみの JSON が、非圧縮形式
    /// (空白・改行・インデントなどを含み整形されている状態) で出力されます。
    ///
    /// @param[in] aStageIndex ステージ番号。有効な範囲の番号が指定される必要があります。
    void Record::dumpJsonStage(int aStageIndex)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aStageIndex, 0, Parameter::GameStageCount);
        mStage[aStageIndex].dumpJson(false);
    }

    //------------------------------------------------------------------------------
    /// ゲームの全情報を含む JSON データを出力します。
    /// この関数を利用して出力したデータはビューアに渡すことが出来ます。
    ///
    /// @param[in] isCompressed 圧縮した形で出力するかどうか。
    ///                         @c true を指定した場合、空白やインデントが取り除かれ、サイズが削減された
    ///                         形で出力されます。
    void Record::dumpJson(bool isCompressed)const
    {
        HPC_PRINT("[");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            // 基本情報
            HPC_PRINT_JSON_DEBUG(!isCompressed, "    "); // インデント (4)
            HPC_PRINT("[");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // 忍者半径
                HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
                HPC_PRINT("%7.3f,", Parameter::CharaRadius());
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

                // 必要周回数
                HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
                HPC_PRINT("%d", Parameter::StageRoundCount);
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            HPC_PRINT_JSON_DEBUG(!isCompressed, "    "); // インデント (4)
            HPC_PRINT("],");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            // ステージ情報表示
            HPC_PRINT_JSON_DEBUG(!isCompressed, "    "); // インデント (4)
            HPC_PRINT("[");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

            for (int index = 0; index < Parameter::GameStageCount; ++index) {
                mStage[index].dumpJson(isCompressed);
                if (index + 1 < Parameter::GameStageCount) {
                    HPC_PRINT(",");
                }
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
            }

            HPC_PRINT_JSON_DEBUG(!isCompressed, "    "); // インデント (4)
            HPC_PRINT("]");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
        HPC_PRINT("]\n");
    }
}

//------------------------------------------------------------------------------
// EOF
