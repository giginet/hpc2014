//------------------------------------------------------------------------------
/// @file
/// @brief    Parameter クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief ゲームに関するパラメータを表します。
    class Parameter
    {
    public:
        ///@name ゲームルール
        //@{
        static const int GameTurnPerStage = 3000;       ///< 1ステージ辺りのゲーム時間
        static const int GameStageCount = 100;          ///< ステージ数
        static const int GameTimeLimitSec = 20;         ///< 制限時間(秒)
        //@}

        ///@name ステージ
        //@{
        static const int StageRoundCount = 3;           ///< 周回数
        //@}

        ///@name キャラ
        //@{
        static const int CharaCountMin = 2;             ///< キャラ最小数(プレイヤー含む)
        static const int CharaCountMax = 4;             ///< キャラ最大数(プレイヤー含む)
        static const int CharaAccelCountMax = 9;        ///< 加速回数の最大数
        static const int CharaInitAccelCount = 9;       ///< 加速回数の初期値
        static float CharaAccelSpeed();                 ///< 加速度
        static float CharaDecelSpeed();                 ///< 1ターン当たりの減速度
        static float CharaRadius();                     ///< 半径
        static float CharaReflectionFactor();           ///< 反発係数
        //@}

        ///@name キャラ（ただし、CPUは異なる場合がある）
        //@{
        static const int CharaAddAccelWaitTurn = 10;    ///< 加速回数が増えるまでのターン数
        //@}
        
        ///@name 蓮
        //@{
        static const int LotusCountMax = 20;            ///< 最大数
        static float LotusBaseRadius();                 ///< 基準となる半径
        //@}

    private:
        Parameter();
    };
}
//------------------------------------------------------------------------------
// EOF
