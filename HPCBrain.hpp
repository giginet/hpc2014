//------------------------------------------------------------------------------
/// @file
/// @brief    Brain クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCAction.hpp"
#include "HPCCharaParam.hpp"

namespace hpc {

    class Random;
    class StageAccessor;
    
    //------------------------------------------------------------------------------
    /// キャラの動作を決定します。
    class Brain
    {
    public:
        Brain();

        void reset();                                       ///< リセットします。
        void setup(const CharaParam& aCharaParam);          ///< 初期状態を設定します。
        
        void init(const StageAccessor& aStageAccessor);     ///< 準備処理を行います。
        /// 次の動作を返します。
        Action getNextAction(
            const StageAccessor& aStageAccessor
            , Random& aRandom
            );

    private:
        CharaParam mCharaParam;     ///< キャラのパラメータ
        int mCpuSaveAccelTurn;      ///< 加速を節約して待機したターン数(CPU)
        
        void initCpu(const StageAccessor& aStageAccessor);  ///< 準備処理を行います。(CPU)
        /// 次の動作を返します。(CPU)
        Action getCpuNextAction(
            const StageAccessor& aStageAccessor
            , Random& aRandom
            );
    };
}
//------------------------------------------------------------------------------
// EOF
