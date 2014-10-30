//------------------------------------------------------------------------------
/// @file
/// @brief    LevelGrid クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCIntVec2.hpp"
#include "HPCRandom.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// マップ生成のためのグリッドを提供します。
    ///
    /// グリッドの位置は左下を原点とした (0, 0) からはじまる xy 要素で表されます。
    /// 但し、内部データとしては各セルは通し番号をもつ一次元の配列として保持されます。
    class LevelGrid
    {
    public:
        LevelGrid(const IntVec2& aSize, int aInhibitMargin, Random& aRand);

        void setOccupied(int x, int y);                             ///< 一点を使用状態にマークします。
        void setOccupied(int x, int y, int width, int height);      ///< 矩形を使用状態にマークします。
        IntVec2 setRandomOccupied();                                ///< 開いているところ一点をランダムでマークします。
        IntVec2 setRandomOccupied(int width, int height);           ///< 開いているところを指定サイズの矩形でランダムにマークします。
        bool isAvailable(int x, int y)const;                        ///< 一点が空いているかを返します。
        bool isAvailable(int x, int y, int width, int hegith)const; ///< 指定サイズの矩形が空いているかを返します。

    private:
        static const int CellSizeMax = 26; ///< グリッドの最大数

        /// グリッドの各セル情報を表します。
        struct Cell {
            Cell();
            
            int index;              ///< セルのインデックス (通し番号)
            IntVec2 pos;            ///< セルの位置
            bool isOccupied;        ///< 占有されているかどうか
        };

        const IntVec2 mSize;        ///< 縦横の長さ
        const int mSurface;         ///< 面積 (セルの総数)。
        Random& mRandom;
        Cell mCells[CellSizeMax * CellSizeMax];          ///< グリッドの各セル
        Cell* mRandArray[CellSizeMax * CellSizeMax];     ///< ランダムな並び順。

        int findAvailableRandCell(int width, int height)const;  ///< 利用可能なセルを探します。
        int axisToIndex(int x, int y)const;                     ///< 座標をインデックスに変換します。
        IntVec2 indexToAxis(int index)const;                    ///< インデックスを座標に変換します。
    };
}
//------------------------------------------------------------------------------
// EOF
