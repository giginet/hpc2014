//------------------------------------------------------------------------------
/// @file
/// @brief    StageAccessor クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCEnemyAccessor.hpp"
#include "HPCField.hpp"
#include "HPCLotusCollection.hpp"

namespace hpc {

    class Chara;
    class Stage;
    
    //------------------------------------------------------------------------------
    /// ステージ情報へのアクセスを提供します。
    /// キャラ（参加者、CPU）には、このクラスのconst参照が渡されます。
    ///
    /// キャラによってplayerやenemiesに該当する対象が違う為、
    /// 各キャラがこのクラスを保持しています。
    class StageAccessor
    {
    public:
        StageAccessor();

        void init(const Stage& aStage, int aPlayerIndex);    ///< 初期設定を行います。
        
        /// @name 各要素へのアクセス
        //@{
        const Chara& player()const;                 ///< プレイヤー情報を返します。
        const EnemyAccessor& enemies()const;        ///< 敵キャラ情報を返します。
        const LotusCollection& lotuses()const;      ///< 蓮情報を返します。
        const Field& field()const;                  ///< フィールド情報を返します。
        //@}

    private:
        const Stage* mStagePtr;         ///< Stageクラスへのポインタ
        EnemyAccessor mEnemyAccessor;   ///< 敵キャラへのアクセサ
        int mPlayerIndex;               ///< プレイヤーのインデックス
    };
}
//------------------------------------------------------------------------------
// EOF
