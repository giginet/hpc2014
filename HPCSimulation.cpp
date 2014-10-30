//------------------------------------------------------------------------------
/// @file
/// @brief    HPCSimulation.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCSimulation.hpp"

#include <cstring>
#include <cstdlib>
#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCTimer.hpp"

namespace {
    /// 入力を受けるコマンド
    enum Command {
        Command_Debug,          ///< デバッガ起動
        Command_OutputJson,     ///< JSON 出力
        Command_Exit,           ///< 終わる
    };

    //------------------------------------------------------------------------------
    /// 入力を受けます。
    Command SelectInput() 
    {
        while (true) {
            HPC_PRINT("[d]ebug | output [j]son | [e]xit: ");

            // 入力待ち
            const char key = getchar();
            if (key == 0x0a) {
                return Command_Exit;
            }
            while (getchar() != 0x0a){} // 改行待ち

            switch (key)
            {
            case 'd': return Command_Debug;
            case 'j': return Command_OutputJson;
            case 'e': return Command_Exit;
            default:
                break;
            }
        }
        return Command_Exit;
    }

    /// 入力を受けるコマンド
    enum DebugCommand {
        DebugCommand_Next,          ///< 次へ
        DebugCommand_Prev,          ///< 前へ
        DebugCommand_Jump,          ///< 指定番号のステージにジャンプ
        DebugCommand_Show,          ///< 再度
        DebugCommand_Help,          ///< ヘルプを表示
        DebugCommand_Exit,          ///< 終わる

        DebugCommand_TERM
    };

    /// 入力を受けるコマンドのセット
    struct DebugCommandSet 
    {
        DebugCommand command;
        int arg1;
        int arg2;

        DebugCommandSet()
            : command(DebugCommand_TERM)
            , arg1(0)
            , arg2(0)
        {
        }

        DebugCommandSet(DebugCommand aCmd, int aArg1, int aArg2)
            : command(aCmd)
            , arg1(aArg1)
            , arg2(aArg2)
        {
            HPC_ENUM_ASSERT(DebugCommand, aCmd);
        }
    };

    //------------------------------------------------------------------------------
    /// 入力を受けます。
    ///
    /// 入力は次の3要素から構成されます。
    /// - コマンド
    /// - 引数1
    /// - 引数2
    ///
    /// 引数1, 引数2 を省略した場合は、0 が設定されます。
    DebugCommandSet SelectInputDebugger(int stage) 
    {
        while (true) {
            HPC_PRINT("[Stage: %d ('h' for help)]> ", stage);

            // 入力待ち
            static const int LineBufferSize = 20; // 20文字くらいあれば十分
            char line[LineBufferSize];
            {
                char* ptr = std::fgets(line, LineBufferSize, stdin);
                (void)ptr;
            }

            // コマンドを読む
            const char* cmdStr = std::strtok(line, " \n\0");
            if (!cmdStr) {
                continue;
            }

            const char* arg1Str = std::strtok(0, " \n\0");
            const char* arg2Str = std::strtok(0, " \n\0");

            const int arg1 = arg1Str ? std::atoi(arg1Str) : 0;
            const int arg2 = arg2Str ? std::atoi(arg2Str) : 0;

            switch (cmdStr[0])
            {
            case 'n': return DebugCommandSet(DebugCommand_Next, arg1, arg2);
            case 'p': return DebugCommandSet(DebugCommand_Prev, arg1, arg2);
            case 'd': return DebugCommandSet(DebugCommand_Show, arg1, arg2);
            case 'j': return DebugCommandSet(DebugCommand_Jump, arg1, arg2);
            case 'h': return DebugCommandSet(DebugCommand_Help, arg1, arg2);
            case 'e': return DebugCommandSet(DebugCommand_Exit, arg1, arg2);
            default:
                break;
            }
        }
        return DebugCommandSet(DebugCommand_Next, 0, 0);
    }

    //------------------------------------------------------------------------------
    /// デバッガのヘルプを表示します。
    void ShowHelp()
    {
        HPC_PRINT(" n           : Go to the next stage.\n");
        HPC_PRINT(" p           : Go to the prev stage.\n");
        HPC_PRINT(" j [stage=0] : Go to the designated stage.\n");
        HPC_PRINT(" d           : Show the result of this stage.\n");
        HPC_PRINT(" h           : Show Help.\n");
        HPC_PRINT(" e           : Exit debugger.\n");
    }
}

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief Simulation クラスのインスタンスを生成します。
    Simulation::Simulation() 
        : mRandSet()
        , mGame(mRandSet)
        , mTimer(Parameter::GameTimeLimitSec)
    {
    }

    //------------------------------------------------------------------------------
    /// @brief ゲームを実行します。
    void Simulation::run()
    {
        // 制限時間と制限ターン数
        mTimer.start();
        while (mGame.isValidStage()) {
            mGame.startStage();
            while (mGame.state() == StageState_Playing && mTimer.isInTime()) {
                mGame.runTurn();
            }
            mGame.onStageDone();
        }
    }

    //------------------------------------------------------------------------------
    /// 結果を表示します。
    void Simulation::outputResult()const
    {
        HPC_PRINT("Done.\n");
        HPC_PRINT("%8s:%8d\n", "Score", mGame.record().score());
        HPC_PRINT("%8s:%8.4f\n", "Time", mTimer.pastSecForPrint());
    }

    //------------------------------------------------------------------------------
    /// @brief ゲームをデバッグ実行します。
    void Simulation::debug()
    {
        // デバッグ前にデバッグをするかどうかを判断する。
        // 入力待ち
        switch (SelectInput()) {
        case Command_Debug:
            runDebugger();
            break;

        case Command_OutputJson:
            mGame.record().dumpJson(true);
            break;

        case Command_Exit:
            break;

        default:
            HPC_SHOULD_NOT_REACH_HERE();
            break;
        }
    }

    //------------------------------------------------------------------------------
    /// JSON データを出力します。
    void Simulation::outputJson(bool isCompressed)const
    {
        mGame.record().dumpJson(isCompressed);
    }

    //------------------------------------------------------------------------------
    /// デバッグ実行を行います。
    void Simulation::runDebugger()
    {
        int stage = 0;
        bool doInput = true;    // ステージ終了時に入力待ち　するか。
        do {
            if (doInput) {
                const DebugCommandSet commandSet = SelectInputDebugger(stage);
                switch(commandSet.command) {
                case DebugCommand_Next:
                    ++stage;
                    break;

                case DebugCommand_Prev:
                    stage = Math::Max(stage - 1, 0);
                    break;

                case DebugCommand_Show:
                    mGame.record().dumpStage(stage);
                    break;

                case DebugCommand_Jump:
                    stage = Math::LimitMinMax(commandSet.arg1, 0, Parameter::GameStageCount - 1);
                    break;

                case DebugCommand_Help:
                    ShowHelp();
                    break;

                case DebugCommand_Exit:
                    stage = Parameter::GameStageCount;
                    break;

                default:
                    HPC_SHOULD_NOT_REACH_HERE();
                    break;
                }
            }
            else {
                ++stage;
            }
        } while (stage < Parameter::GameStageCount);
    }
}

//------------------------------------------------------------------------------
// EOF
