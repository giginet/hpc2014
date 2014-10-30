//------------------------------------------------------------------------------
/// @file
/// @brief    HPCTurnResult.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCTurnResult.hpp"

#include "HPCCommon.hpp"
#include "HPCLevelDesigner.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    TurnResult::TurnResult()
        : charas()
        , state(StageState_TERM)
    {
        reset();
    }

    //------------------------------------------------------------------------------
    /// 値を既定の値で初期化します。
    void TurnResult::reset()
    {
        for (int index = 0; index < Parameter::CharaCountMax; ++index) {
            charas[index].pos.reset();
            charas[index].accelCount = 0;
            charas[index].passedLotusCount = 0;
        }
        
        state = StageState_TERM;
    }

    //------------------------------------------------------------------------------
    /// 別の TurnResult で値を上書きします。
    /// 基本情報がすべて上書きされます。
    ///
    /// @param[in] aResult  上書きする TurnResult
    void TurnResult::set(const TurnResult& aResult)
    {
        for (int index = 0; index < Parameter::CharaCountMax; ++index) {
            charas[index] = aResult.charas[index];
        }
        state = aResult.state;
    }
}

//------------------------------------------------------------------------------
// EOF
