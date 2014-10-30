//------------------------------------------------------------------------------
/// @file
/// @brief    CharaCollection クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCChara.hpp"
#include "HPCParameter.hpp"
#include "HPCVec2.hpp"

namespace hpc {

    class Random;
    class Stage;
    
    //------------------------------------------------------------------------------
    /// キャラの組を表します。
    ///
    /// 1ステージのすべてのキャラは、この CharaCollection クラスに格納されます。
    /// このクラスは、回答者には公開されません。
    class CharaCollection
    {
    public:
        CharaCollection();

        void procDecideAction(Random& aRandom);         ///< 動作を決定します。
        void procExecAction();                          ///< 動作を実行します。
        void procCheckColl();                           ///< キャラ同士の衝突判定を行います。
        void procEnd(const Stage& aStage);              ///< 最終処理を行います。
        
        void reset();                                   ///< キャラデータを初期化します。
        /// キャラデータ追加します。
        void setupAddChara(
            const Vec2& aCharaPos
            , const CharaParam& aCharaParam
            );
        int count()const;                               ///< 有効なキャラ数を返します。
        bool isAllHumanGoal()const;                     ///< 人間キャラが全員ゴールしたかどうかを返します。
        int goalCount()const;                           ///< ゴールしたキャラ数を返します。

        /// @name 有効なキャラへのアクセス
        //@{
        const Chara& operator[](int aIndex)const;
        Chara& operator[](int aIndex);
        //@}

    private:
        Chara mCharas[Parameter::CharaCountMax];        ///< キャラ用配列
        CharaType mCharaTypes[Parameter::CharaCountMax];///< キャラの種類
        int mCount;                                     ///< 有効なキャラ数
        
        void updateRank();
    };
}
//------------------------------------------------------------------------------
// EOF
