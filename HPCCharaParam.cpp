//------------------------------------------------------------------------------
/// @file
/// @brief    HPCCharaParam.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCCharaParam.hpp"

#include "HPCCommon.hpp"

namespace hpc {
    //------------------------------------------------------------------------------
    /// Human用としてインスタンスを生成します。
    const CharaParam CharaParam::CreateHuman()
    {
        CharaParam param;
        param.mType = CharaType_Human;
        param.mStrength = 0;
        
        return param;
    }

    //------------------------------------------------------------------------------
    /// CPU用としてインスタンスを生成します。
    ///
    /// @param[in] aStrength     強さ
    const CharaParam CharaParam::CreateCpu(
        int aStrength
        )
    {
        CharaParam param;
        param.mType = CharaType_Cpu;
        param.mStrength = aStrength;
        
        return param;
    }
    
    //------------------------------------------------------------------------------
    /// インスタンスを生成します。
    CharaParam::CharaParam()
        : mType(CharaType_TERM)
        , mStrength(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 値を初期化します。
    void CharaParam::reset()
    {
        mType = CharaType_TERM;
        mStrength = 0;
    }

    //------------------------------------------------------------------------------
    /// @return キャラの種類。
    CharaType CharaParam::type()const
    {
        return mType;
    }

    //------------------------------------------------------------------------------
    /// @return 強さ。
    int CharaParam::strength()const
    {
        return mStrength;
    }
}

//------------------------------------------------------------------------------
// EOF
