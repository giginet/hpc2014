//------------------------------------------------------------------------------
/// @file
/// @brief    HPCCircle.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCCircle.hpp"
#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 初期値を指定してクラスのインスタンスを生成します。
    ///
    /// @param[in] aPos    円の中心を表す座標。
    /// @param[in] aRadius 半径。正の値を設定する必要があります。
    Circle::Circle(const Vec2& aPos, float aRadius)
        : mPos(aPos)
        , mRadius(aRadius)
    {
        HPC_LB_ASSERT_F(aRadius, 0.0f);
    }

    //------------------------------------------------------------------------------
    /// 中心位置と半径を与えて円を設定します。
    ///
    /// @param[in] aPos    円の中心を表す座標。
    /// @param[in] aRadius 半径。正の値を設定する必要があります。
    void Circle::setup(const Vec2& aPos, float aRadius)
    {
        HPC_LB_ASSERT_F(aRadius, 0.0f);

        mPos = aPos;
        mRadius = aRadius;
    }

    //------------------------------------------------------------------------------
    /// 半径をそのままにして、中心座標だけを設定します。
    ///
    /// @param[in] aPos 設定する中心座標。
    void Circle::setPos(const Vec2& aPos)
    {
        mPos = aPos;
    }

    //------------------------------------------------------------------------------
    /// 中心座標をそのままにして、半径だけを設定します。
    ///
    /// @param[in] aRadius 円の半径。正の値を設定する必要があります。
    void Circle::setRadius(float aRadius)
    {
        HPC_LB_ASSERT_F(aRadius, 0.0f);
        mRadius = aRadius;
    }

    //------------------------------------------------------------------------------
    /// 中心位置を与えられた量だけ平行移動します。
    ///
    /// @param[in] aVec 平行移動する量。
    void Circle::move(const Vec2& aVec)
    {
        mPos += aVec;
    }

    //------------------------------------------------------------------------------
    const Vec2& Circle::pos()const
    {
        return mPos;
    }

    //------------------------------------------------------------------------------
    float Circle::radius()const
    {
        return mRadius;
    }
}

//------------------------------------------------------------------------------
// EOF
