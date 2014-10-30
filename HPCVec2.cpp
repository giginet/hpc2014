//------------------------------------------------------------------------------
/// @file
/// @brief    HPCVec2.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCVec2.hpp"

#include "HPCCommon.hpp"
#include "HPCMath.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 生成されたオブジェクトが、原点またはゼロベクトルを表すように
    /// ゼロで初期化されたインスタンスを生成します。
    Vec2::Vec2()
        : x(0.0f)
        , y(0.0f)
    {
        reset();
    }

    //------------------------------------------------------------------------------
    /// x, y の各成分を定めてクラスのインスタンスを生成します。
    ///
    /// @param[in] aX  x成分の値
    /// @param[in] aY  y成分の値
    Vec2::Vec2(float aX, float aY)
        : x(aX)
        , y(aY)
    {
    }

    //------------------------------------------------------------------------------
    /// 値を (0, 0) に初期化します。
    void Vec2::reset()
    {
        x = 0.0f;
        y = 0.0f;
    }

    //------------------------------------------------------------------------------
    /// 現在の値がゼロベクトル（厳密）かどうかを返します。
    ///
    /// @return x, y がそれぞれ厳密にゼロであれば @c true を返し
    ///         そうでなければ @c false を返します。
    bool Vec2::isZero()const
    {
        return x == 0.0f && y == 0.0f;
    }

    //------------------------------------------------------------------------------
    /// 引数に渡すベクトルと現在の値を比較し、誤差の範囲内で等しいかを返します。
    ///
    /// @param[in] aVec 比較対象とするベクトル。
    ///
    /// @return aVec の各要素が x, y と誤差の範囲で等しければ @c true を返し
    ///         そうでなければ @c false を返します。
    bool Vec2::equals(const Vec2& aVec)const
    {
        return Math::IsEqual(x, aVec.x) && Math::IsEqual(y, aVec.y);
    }

    //------------------------------------------------------------------------------
    /// 値の表す座標 (x, y) と引数 aPoint の表す座標間の距離を返します。
    ///
    /// @param[in] aPoint 距離計算の目的地となる座標。
    ///
    /// @return (x, y) と aPoint の距離。
    float Vec2::dist(const Vec2& aPoint)const
    {
        return Math::Sqrt(squareDist(aPoint));
    }

    //------------------------------------------------------------------------------
    /// 値の表す座標 (x, y) と引数の表す座標間の距離の二乗を返します。
    ///
    /// @param[in] aPoint 距離計算の目的地となる座標。
    ///
    /// @return (x, y) と aPoint の距離の二乗。
    float Vec2::squareDist(const Vec2& aPoint)const
    {
        const float x2 = (aPoint.x - x) * (aPoint.x - x);
        const float y2 = (aPoint.y - y) * (aPoint.y - y);

        return x2 + y2;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) と 引数 aVec との内積を計算します。
    ///
    /// @param[in] aVec 内積計算の対象となるベクトル。
    ///
    /// @return (x, y) と aVec の内積。
    float Vec2::dot(const Vec2& aVec)const
    {
        return x * aVec.x + y * aVec.y;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) と 引数 aVec との外積を計算します。
    ///
    /// @param[in] aVec 外積計算の対象となるベクトル。
    ///
    /// @return (x, y) と aVec の外積。
    float Vec2::cross(const Vec2& aVec)const
    {
        return x * aVec.y - y * aVec.x;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) と引数 aVec の表すベクトルとの
    /// なす角の余弦を求めます。
    ///
    /// @param[in] aVec なす角を計算するためのベクトル。
    ///
    /// @return (x, y) と aVec とのなす角 (ラジアン) の余弦 (cos)。
    float Vec2::cos(const Vec2& aVec)const
    {
        const float d = dot(aVec);
        const float l = length() * aVec.length();

        HPC_LB_ASSERT_F(l, 0.0f);
        return d / l;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) と引数 aVec の表すベクトルとの
    /// なす角を求めます。
    ///
    /// @param[in] aVec なす角を計算するためのベクトル。
    ///
    /// @return (x, y) と aVec とのなす角の大きさ (ラジアン)。
    float Vec2::angle(const Vec2& aVec)const
    {
        return Math::ACos(cos(aVec));
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトルを aTarget と同じ方向に回転するとき、
    /// その回転に必要な角度を、向きを含めて計算します。
    /// 計算結果は angle に正負いずれかの符号がついたものになります。
    /// 
    /// @note 向きは反時計回りが正、時計回りが負を表します。
    ///
    /// @param[in] aTarget 目標とする方向を表すベクトル。
    ///
    /// @return 回転に必要な角度 (ラジアン) が [-π..π] の範囲で返されます。
    float Vec2::rotSign(const Vec2& aTarget)const
    {
        const float d = dot(aTarget);
        const float c = x * aTarget.y - y * aTarget.x;

        return Math::ATan2(c, d);
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) の長さの二乗を計算します。
    ///
    /// @return ベクトルの長さの二乗。
    float Vec2::squareLength()const
    {
        return x * x + y * y;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) の長さを計算します。
    ///
    /// @return ベクトルの長さ。
    float Vec2::length()const
    {
        return Math::Sqrt(squareLength());
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) を指定の大きさに設定します。
    ///
    /// @pre ベクトルがゼロベクトルではないことをあらかじめ確認する必要があります。
    ///
    /// @param[in] aLength ベクトルの長さ。正の数を設定します。
    void Vec2::normalize(float aLength)
    {
        HPC_LB_ASSERT_F(aLength, 0.0f);

        *this /= length();
        *this *= aLength;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) を指定の大きさに設定します。
    ///
    /// @pre ベクトルがゼロベクトルではないことをあらかじめ確認する必要があります。
    ///
    /// @param[in] aLength ベクトルの長さ。正の数を設定します。
    ///
    /// @return (x, y) を aLength の長さに設定したベクトル。
    Vec2 Vec2::getNormalized(float aLength)const
    {
        HPC_LB_ASSERT_F(aLength, 0.0f);

        Vec2 aVec(*this);
        aVec.normalize(aLength);
        return aVec;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) を引数に与えられた角度方向に回転させます。
    ///
    /// @param[in] aRad 回転角 (ラジアン)。反時計回りが正の値を表します。
    void Vec2::rotate(float aRad)
    {
        const float rotatedX = Math::Cos(aRad) * x - Math::Sin(aRad) * y;
        const float rotatedY = Math::Sin(aRad) * x + Math::Cos(aRad) * y;

        x = rotatedX;
        y = rotatedY;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) を引数に与えられた角度方向に回転させた
    /// ベクトルを返します。
    /// インスタンスの値自体は変更されません。
    ///
    /// @param[in] aRad 回転角 (ラジアン)。反時計回りが正の値を表します。
    ///
    /// @return (x, y) を aRad だけ回転させて得られるベクトル。
    Vec2 Vec2::getRotated(float aRad)const
    {
        Vec2 v(*this);
        v.rotate(aRad);

        return v;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) を引数に与えられたベクトルに射影します。
    ///
    /// @param[in] aVec 射影対象。求めたベクトルはこのベクトルと平行になります。
    void Vec2::project(const Vec2& aVec)
    {
        const float l = aVec.length();
        if (l == 0.0f) {
            HPC_SHOULD_NOT_REACH_HERE();
            return;
        }

        const float len = this->dot(aVec) / l;
        Vec2 result = aVec.getNormalized() * len;
        x = result.x;
        y = result.y;
    }

    //------------------------------------------------------------------------------
    /// 値の表すベクトル (x, y) を引数に与えられたベクトルに射影した
    /// ベクトルを返します。
    /// インスタンスの値自体は変更されません。
    ///
    /// @param[in] aVec 射影対象。生成されるベクトルはこのベクトルと平行になります。
    ///
    /// @return (x, y) を aVec に射影したベクトル。
    Vec2 Vec2::getProjected(const Vec2& aVec)const
    {
        Vec2 v(*this);
        v.project(aVec);
        return v;
    }

    //------------------------------------------------------------------------------
    bool Vec2::operator==(const Vec2& aRhs)const
    {
        return x == aRhs.x && y == aRhs.y;
    }

    //------------------------------------------------------------------------------
    Vec2 Vec2::operator-()const
    {
        return Vec2(-x, -y);
    }

    //------------------------------------------------------------------------------
    Vec2& Vec2::operator+=(const Vec2& aRhs)
    {
        x += aRhs.x;
        y += aRhs.y;

        return *this;
    }

    //------------------------------------------------------------------------------
    Vec2& Vec2::operator-=(const Vec2& aRhs)
    {
        return *this += (-aRhs);
    }

    //------------------------------------------------------------------------------
    Vec2 Vec2::operator+(const Vec2& aRhs)const
    {
        Vec2 v(*this);
        v += aRhs;

        return v;
    }

    //------------------------------------------------------------------------------
    Vec2 Vec2::operator-(const Vec2& aRhs)const
    {
        return *this + (-aRhs);
    }

    //------------------------------------------------------------------------------
    Vec2& Vec2::operator*=(float aRhs)
    {
        x *= aRhs;
        y *= aRhs;

        return *this;
    }

    //------------------------------------------------------------------------------
    Vec2& Vec2::operator/=(float aRhs)
    {
        x /= aRhs;
        y /= aRhs;

        return *this;
    }
    
    //------------------------------------------------------------------------------
    Vec2 Vec2::operator*(float aRhs)const
    {
        Vec2 v(*this);
        v *= aRhs;
        return v;
    }

    //------------------------------------------------------------------------------
    Vec2 Vec2::operator/(float aRhs)const
    {
        Vec2 v(*this);
        v /= aRhs;
        return v;
    }

    //------------------------------------------------------------------------------
    Vec2 operator*(float aLhs, const Vec2& aRhs) 
    {
        return aRhs * aLhs;
    }
}

//------------------------------------------------------------------------------
// EOF
