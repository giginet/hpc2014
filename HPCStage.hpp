//------------------------------------------------------------------------------
/// @file
/// @brief    Stage クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCCharaCollection.hpp"
#include "HPCField.hpp"
#include "HPCLotusCollection.hpp"
#include "HPCTurnResult.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ゲームの1ステージを表します。
    class Stage 
    {
    public:
        Stage();

        void reset();                                   ///< ステージ情報を削除します。

        ///@name ステージの実行
        //@{
        void start();                                   ///< ステージを開始します。
        void runTurn(Random& aRandom);                  ///< ターンを1つ進めます。
        const TurnResult& lastTurnResult()const;        ///< 最後のターン実行後の結果を返します。
        //@}

        /// @name 各要素へのアクセス
        //@{
        const CharaCollection& charas()const;       ///< キャラ情報を返します。
        CharaCollection& charas();                  ///< キャラ情報を返します。
        const LotusCollection& lotuses()const;      ///< 蓮情報を返します。
        LotusCollection& lotuses();                 ///< 蓮情報を返します。
        const Field& field()const;                  ///< フィールド情報を返します。
        Field& field();                             ///< フィールド情報を返します。
        //@}

    private:
        CharaCollection mCharas;        ///< キャラ情報
        LotusCollection mLotuses;       ///< 蓮情報
        Field mField;                   ///< フィールド情報
        TurnResult mTurnResult;         ///< ターンの実行結果
        int mTurnIndex;                 ///< 現在のターン番号

        void updateTurnResult();    ///< TurnResultを更新します。
    };
}
//------------------------------------------------------------------------------
// EOF
