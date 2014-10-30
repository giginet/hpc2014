//------------------------------------------------------------------------------
/// @file
/// @brief    HPCLevelGrid.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCLevelGrid.hpp"

#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCRandom.hpp"


namespace {
    using namespace hpc;

    //------------------------------------------------------------------------------
    /// 2つの値を交換します。
    ///
    /// @tparam T 交換する値の型
    /// @param[in,out] aLhs 左辺
    /// @param[in,out] aRhs 右辺
    template<class T>
    void Swap(T& aLhs, T& aRhs)
    {
        T temp = aLhs;
        aLhs = aRhs;
        aRhs = temp;
    }

    //------------------------------------------------------------------------------
    /// 配列をシャッフルします。
    ///
    /// @tparam    T シャッフルする配列の要素の型
    /// @param[in,out] aArray シャッフルする配列
    /// @param[in]     aCount aArray の有効な要素数
    /// @param[in,out] aRand 乱数オブジェクト
    template<class T>
    void ShuffleArray(T aArray[], int aCount, Random& aRand)
    {
        HPC_LB_ASSERT_I(aCount, 0);
        for (int index = 0; index < aCount - 1; ++index) {
            const int randIndex = aRand.randMinTerm(index, aCount);
            Swap(aArray[index], aArray[randIndex]);
        }
    }
}

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    /// 値はすべてゼロで初期化されます。
    LevelGrid::Cell::Cell()
        : index(0)
        , pos()
        , isOccupied(false)
    {
    }

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    /// @param[in]     aSize グリッドのサイズ。
    /// @param[in]     aInhibitMargin 領域確保禁止マージン。
    /// @param[in,out] aRand 乱数オブジェクト。
    LevelGrid::LevelGrid(const IntVec2& aSize, const int aInhibitMargin, Random& aRand)
        : mSize(aSize)
        , mSurface(mSize.x * mSize.y)
        , mRandom(aRand)
    {
        HPC_RANGE_ASSERT_MIN_MAX_I(mSize.x, 1, CellSizeMax);
        HPC_RANGE_ASSERT_MIN_MAX_I(mSize.y, 1, CellSizeMax);

        for (int index = 0; index < mSurface; ++index) {
            mCells[index].pos = indexToAxis(index);
            mCells[index].index = index;
            mRandArray[index] = &mCells[index];
        }
        // 乱数配列の生成
        ShuffleArray(mRandArray, mSurface, mRandom);
        
        // 領域確保禁止マージンに該当する範囲は、確保済みに設定しておく
        for (int ix = 0; ix < mSize.x; ++ix) {
            for (int iy = 0; iy < mSize.y; ++iy) {
                if (
                    aInhibitMargin <= ix
                    && ix < mSize.x - aInhibitMargin
                    && aInhibitMargin <= iy
                    && iy < mSize.y - aInhibitMargin
                ) {
                    continue;
                }
                
                setOccupied(ix, iy);
            }
        }
    }

    //------------------------------------------------------------------------------
    /// 引数に与えた一点の示すセルを使用中にします。
    ///
    /// @note セルが使用中かどうかは検査しません。
    ///       必要に応じて、isAvailable(int,int) 関数などで検査してください。
    ///
    /// @param[in] aX セルの x 座標
    /// @param[in] aY セルの y 座標
    void LevelGrid::setOccupied(int aX, int aY)
    {
        const int index = axisToIndex(aX, aY);
        HPC_ASSERT(mCells[index].pos.x == aX);
        HPC_ASSERT(mCells[index].pos.y == aY);

        mCells[index].isOccupied = true;
    }

    //------------------------------------------------------------------------------
    /// 引数に与えた矩形の示すセル群を使用中にします。
    ///
    /// @note セルが使用中かどうかは検査しません。
    ///       必要に応じて、isAvailable(int,int,int,int) 関数などで検査してください。
    ///
    /// @param[in] aX      セルの x 座標
    /// @param[in] aY      セルの y 座標
    /// @param[in] aWidth  占有するセルの横幅。
    /// @param[in] aHeight 占有するセルの縦幅。。
    void LevelGrid::setOccupied(int aX, int aY, int aWidth, int aHeight)
    {
        for (int ix = aX; ix < aX + aWidth; ++ix) {
            for (int iy = aY; iy < aY + aHeight; ++iy) {
                setOccupied(ix, iy);
            }
        }
    }
    
    //------------------------------------------------------------------------------
    /// 引数に与えた aWidth × aHeight の矩形に対し、
    /// セルを確保できるかどうか検査し、確保可能な場合はその左下を表すインデックスを返します。
    ///
    /// @param[in] aWidth   占有する横幅。
    /// @param[in] aHeight  占有する高さ。
    ///
    /// @return 利用可能な乱数値をもつセルのインデックス。
    int LevelGrid::findAvailableRandCell(int aWidth, int aHeight)const
    {
        for (int index = 0; index < mSurface; ++index) {
            // 利用不可能なら次へ
            if (mRandArray[index]->isOccupied) {
                continue;
            }
            // 占有可能か調べる
            if (isAvailable(mRandArray[index]->pos.x, mRandArray[index]->pos.y, aWidth, aHeight)) {
                return mRandArray[index]->index;
            }
        }
        HPC_SHOULD_NOT_REACH_HERE();
        return 0;
    }

    //------------------------------------------------------------------------------
    /// セルのうち、空いているセルがあれば1つ確保してその位置を返します。
    ///
    /// @return 取得した位置を表す IntVec2 
    IntVec2 LevelGrid::setRandomOccupied()
    {
        const int index = findAvailableRandCell(1, 1);
        HPC_ASSERT(!mCells[index].isOccupied);
        mCells[index].isOccupied = true;

        return mCells[index].pos;
    }

    //------------------------------------------------------------------------------
    /// セルのうち、aWidth × aHeight サイズのセルを確保可能であれば確保して
    /// その位置を返します。
    ///
    /// @param[in] aWidth  確保するセルの横幅。
    /// @param[in] aHeight 確保するセルの高さ。
    ///
    /// @return 取得した箇所の左下端を表す IntVec2
    IntVec2 LevelGrid::setRandomOccupied(int aWidth, int aHeight)
    {
        const int index = findAvailableRandCell(aWidth, aHeight);
        const IntVec2 pos = mCells[index].pos;
        for (int ix = pos.x; ix < pos.x + aWidth; ++ix) {
            for (int iy = pos.y; iy < pos.y + aHeight; ++iy) {
                HPC_RANGE_ASSERT_MIN_UB_I(ix, 0, mSize.x);
                HPC_RANGE_ASSERT_MIN_UB_I(iy, 0, mSize.y);
                const int cellIndex = axisToIndex(ix, iy);
                HPC_ASSERT(!mCells[cellIndex].isOccupied);
                mCells[cellIndex].isOccupied = true;
            }
        }
        return pos;
    }

    //------------------------------------------------------------------------------
    /// 引数に渡した位置 (aX, aY) が利用可能かどうかを返します。
    ///
    /// @param[in] aX 検査する位置の x 座標
    /// @param[in] aY 検査する位置の y 座標
    ///
    /// @return 利用可能なら @c true を返し、そうでなければ @c false を返します。
    bool LevelGrid::isAvailable(int aX, int aY)const
    {
        // 範囲外を指定した場合
        if (aX < 0 || aX >= mSize.x || aY < 0 || aY >= mSize.y) {
            return false;
        }

        const Cell& cell = mCells[axisToIndex(aX, aY)];
        HPC_ASSERT(cell.pos.x == aX);
        HPC_ASSERT(cell.pos.y == aY);

        return !cell.isOccupied;
    }

    //------------------------------------------------------------------------------
    /// 引数に渡した矩形が利用可能かどうかを返します。
    ///
    /// @param[in] aX 検査する矩形の左下 x 座標
    /// @param[in] aY 検査する矩形の左下 y 座標
    /// @param[in] aWidth  検査する矩形の横幅
    /// @param[in] aHeight 検査する矩形の高さ
    ///
    /// @return 利用可能なら @c true を返し、そうでなければ @c false を返します。
    bool LevelGrid::isAvailable(int aX, int aY, int aWidth, int aHeight)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aX, 0, mSize.x);
        HPC_RANGE_ASSERT_MIN_UB_I(aY, 0, mSize.y);
        HPC_LB_ASSERT_I(aWidth, 0);
        HPC_LB_ASSERT_I(aHeight, 0);
        for (int ix = aX; ix < aX + aWidth; ++ix) {
            for (int iy = aY; iy < aY + aHeight; ++iy) {
                if (!isAvailable(ix, iy)) {
                    return false;
                }
            }
        }
        return true;
    }

    //------------------------------------------------------------------------------
    /// 与えられたグリッド上の座標 (aX, aY) を、インデックスに変換します。
    ///
    /// @param[in] aX グリッドの x 座標
    /// @param[in] aY グリッドの y 座標
    ///
    /// @return 座標 (aX, aY) をインデックスに変換した値。
    int LevelGrid::axisToIndex(int aX, int aY)const
    {
        return mSize.x * aY + aX;
    }

    //------------------------------------------------------------------------------
    /// 与えられたインデックスを、グリッド上の座標に変換します。
    ///
    /// @param[in] aIndex グリッドのインデックス。
    ///
    /// @return aIndex を座標に変換した値。
    IntVec2 LevelGrid::indexToAxis(int aIndex)const
    {
        const int x = aIndex % mSize.x;
        const int y = aIndex / mSize.x;

        return IntVec2(x, y);
    }
}

//------------------------------------------------------------------------------
// EOF
