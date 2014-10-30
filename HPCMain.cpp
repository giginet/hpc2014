//------------------------------------------------------------------------------
/// @file
/// @brief    main 関数
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include <cstring>
#include "HPCCommon.hpp"
#include "HPCSimulation.hpp"

//------------------------------------------------------------------------------
namespace {
    /// 引数による操作種類
    enum Operation {
        Operation_Normal,                   ///< 通常動作
        Operation_NoDebug,                  ///< デバッグなし
        Operation_OutputJson,               ///< JSON の出力
        Operation_OutputJsonCompressed,     ///< 圧縮された JSON の出力

        Operation_TERM
    };
    // new, delete を使うことは出来ないので static な変数として
    // Simulation クラスを用意します。
    hpc::Simulation sSim;
}

//------------------------------------------------------------------------------
/// アプリケーションのエントリポイントです。
///
/// @return プログラムが正常に終了したら 0 を返します。
///
/// @note 起動時引数を設定することで、挙動を変更することができます。
///
///   オプション | 説明
///  ------------|----------------------------------------------
///   -n         | デバッグを行いません。
///   -j         | デバッグを行わず、結果を JSON で出力します。
///
int main(int argc, const char* argv[])
{
    Operation operation = Operation_Normal;
    
    // 引数は 1 つまで有効。
    if (argc > 2) {
        HPC_PRINT("Invalid Argument.\n");
        return 0;
    }
    // 引数がある場合、引数を記録する。
    else if (argc > 1) {
        if (!std::strcmp(argv[1], "-n")) {
            operation = Operation_NoDebug;
        }
        else if (!std::strcmp(argv[1], "-j")) {
            operation = Operation_OutputJsonCompressed;
        }
        else if (!std::strcmp(argv[1], "-jd")) {
            operation = Operation_OutputJson;
        }
        else {
            HPC_PRINT("Invalid Argument: %s is unknown command.\n", argv[1]);
            return 0;
        }
    }
    // プログラムの実行
    {
        sSim.run();

        switch (operation) {
        case Operation_Normal:
            sSim.outputResult();
            sSim.debug();
            break;

        case Operation_NoDebug:
            sSim.outputResult();
            break;

        case Operation_OutputJson:
            sSim.outputJson(false);
            break;

        case Operation_OutputJsonCompressed:
            sSim.outputJson(true);
            break;

        default:
            HPC_SHOULD_NOT_REACH_HERE();
            break;
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
// EOF
