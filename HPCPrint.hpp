//------------------------------------------------------------------------------
/// @file
/// @brief    画面出力の定義
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include <cstdio>

/// 文字列を画面に表示します。
///
/// @param[in] ... 書式つき文字列とデータ。 std::printf の記法に準拠します。
#define HPC_PRINT(...) \
    do { std::printf(__VA_ARGS__); } while (false)

/// 項目を付加して画面に文字列を表示します。　
///
/// @param[in] category  項目名。6文字以内。
/// @param[in] ...       書式つき文字列とデータ。 std::printf の記法に準拠します。
#define HPC_PRINT_LOG(category, ...) \
    do { \
        std::printf("[%6s] ", category); \
        std::printf(__VA_ARGS__); \
    } while (false)

/// デバッグ用 JSON 表示専用の画面出力です。
///
/// @param[in] doOutputDebug 出力するかどうか。
/// @param[in] ...           書式つき文字列とデータ。 std::printf の記法に準拠します。
#define HPC_PRINT_JSON_DEBUG(doOutputDebug, ...) \
    do { \
        if (doOutputDebug) { \
            std::printf(__VA_ARGS__); \
        } \
    } while (false)

//------------------------------------------------------------------------------
// EOF
