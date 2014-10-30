#-------------------------------------------------------------------------------
#   HAL Programming Contest 2014 makefile for GCC
#

SourceFiles := $(wildcard *.cpp)
ObjectFiles := $(SourceFiles:%.cpp=%.o)
DependFiles := $(SourceFiles:%.cpp=%.d)
ExecuteFile := ./hpc2014.exe

# Atを@にしておくと、コマンドの実行結果出力を抑止できます。
# 出力が必要な場合は空白を指定します。
At := @
#At := 

# ターゲットを出力する、ユーティリティ。
EchoTarget = @echo '\# $@'

Compiler := g++
Linker := g++

# -Wall : 基本的なワーニングを全て有効に
# -Werror : ワーニングはエラーに
# -Wshadow : ローカルスコープの名前が、外のスコープの名前を隠している時にワーニング
CompileOption := -Wall -Werror -Wshadow -DDEBUG -MMD -O3
LinkOption := 

#-------------------------------------------------------------------------------
.PHONY: all clean run help

all : $(ExecuteFile)

$(ExecuteFile) : $(ObjectFiles)
	$(EchoTarget)
	$(At) $(Linker) $(LinkOption) $(ObjectFiles) -o $(ExecuteFile)

clean :
	$(EchoTarget)
	$(At) rm -fv $(ExecuteFile) $(ObjectFiles) $(DependFiles) $(ExecuteFile).stackdump

run : $(ExecuteFile)
	$(EchoTarget)
	$(At) $(ExecuteFile)

help :
	@echo '--- ターゲット一覧 ---'
	@echo '- all   : 全てをビルドし、実行ファイルを作成する。(デフォルトターゲット)'
	@echo '- clean : 生成物を削除する。'
	@echo '- help  : このメッセージを出力する。'
	@echo '- run   : 実行する。'

%.o : %.cpp Makefile
	$(EchoTarget)
	$(At) $(Compiler) $(CompileOption) -c $< -o $@

#-------------------------------------------------------------------------------
-include $(DependFiles)
