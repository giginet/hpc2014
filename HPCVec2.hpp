//------------------------------------------------------------------------------
/// @file
/// @brief    Vec2 クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

namespace hpc {

    //------------------------------------------------------------------------------
    /// 二次元の値を表します。
    ///
    /// 二次元の要素を表すための型です。
    /// 要素は二次元座標上の点ならびに二次元ベクトルの両方の意味を持ち、
    /// 各オブジェクトのプログラム上での意味に合わせて使い分けます。
    class Vec2
    {
    public:
        Vec2();                                     ///< ゼロベクトルとしてインスタンスを生成します。
        Vec2(float aX, float aY);                   ///< 値を指定してインスタンスを生成します。

        void reset();                               ///< 値をゼロ値で初期化します。

        ///@name 等値比較
        //@{
        bool operator==(const Vec2& aRhs)const;                 ///< 2つのベクトルが等しいかを返します。
        bool isZero()const;                                     ///< ゼロベクトルと等しいかを返します。
        bool equals(const Vec2& aVec)const;                     ///< 2つのベクトルが誤差の範囲内で等しいかを求めます。
        //@}

        ///@name 四則演算
        //@{
        Vec2 operator-()const;                                  ///< 負のベクトルを返します。
        Vec2& operator+=(const Vec2& aRhs);                     ///< ベクトルを加算し結果を代入します。
        Vec2& operator-=(const Vec2& aRhs);                     ///< ベクトルを減算し結果を代入します。
        Vec2& operator*=(float aRhs);                           ///< ベクトルの実数倍を計算し結果を代入します。
        Vec2& operator/=(float aRhs);                           ///< ベクトルを実数で割った結果を代入します。
        Vec2 operator+(const Vec2& aRhs)const;                  ///< 2つのベクトルの和を返します。
        Vec2 operator-(const Vec2& aRhs)const;                  ///< 2つのベクトルの差を返します。
        Vec2 operator*(float aRhs)const;                        ///< ベクトルの実数倍を返します。
        Vec2 operator/(float aRhs)const;                        ///< ベクトルを実数で割った値を返します。
        friend Vec2 operator*(float aLhs, const Vec2& aRhs);    ///< 実数のベクトル倍を返します。
        //@}

        float dot(const Vec2& aVec)const;                       ///< 与えたベクトルとの内積を求めます。
        float cross(const Vec2& aVec)const;                     ///< 与えたベクトルとの外積を求めます。
        float cos(const Vec2& aVec)const;                       ///< 与えたベクトルとの余弦を求めます。
        float angle(const Vec2& aVec)const;                     ///< 与えたベクトルとのなす角を求めます。
        float rotSign(const Vec2& aTarget)const;                ///< 与えたベクトルへの回転量を符号付きで求めます。
        float length()const;                                    ///< ベクトルの長さを求めます。
        float squareLength()const;                              ///< ベクトルの長さの二乗を求めます。
        float dist(const Vec2& aPoint)const;                    ///< 与えた点との距離を求めます。
        float squareDist(const Vec2& aPoint)const;              ///< 与えた点との距離の二乗を求めます。
        void normalize(float aLength = 1.0f);                   ///< 一定の長さに正規化します。
        Vec2 getNormalized(float aLength = 1.0f)const;          ///< 一定の長さに正規化されたベクトルを返します。
        void rotate(float aRad);                                ///< ベクトルを一定角度回転させます。
        Vec2 getRotated(float aRad)const;                       ///< 一定角度回転させたベクトルを返します。
        void project(const Vec2& aVec);                         ///< ベクトルを射影します。
        Vec2 getProjected(const Vec2& aVec)const;               ///< 射影したベクトルを返します。

        float x;    ///< 値の x 要素
        float y;    ///< 値の y 要素
    };
}
//------------------------------------------------------------------------------
// EOF
