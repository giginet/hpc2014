/*
 * HAL Programming Contest 2014 Viewer
 * Copyright (c) 2014 HAL Laboratory, Inc.
 * このファイルの利用は、同梱のREADMEにある利用条件に従ってください
 */
angular.module('viewer', []).config(function ($locationProvider) {
  'use strict';
  $locationProvider.html5Mode(true);
}).controller('ViewerController', function ($scope, $location, $timeout, $window) {
  'use strict';
  var $slider = $('#slider'),
    ninjaColor = ['#0000ff', '#8b4513', '#d2691e', '#b8860b'];
  $slider.slider();

  $scope.isPlay = $scope.isShowInfo = false;
  $scope.wait = 50;
  $scope.currentStageNo = 0;
  $scope.currentTurnNo = 0;
  $scope.isNowLoading = false;
  $scope.ninjaColor = function (i) {
    return ninjaColor[i];
  };

  // グリッド画像生成
  (function (canvas) {
    var ctx = canvas.getContext('2d');
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.strokeStyle = '#aaa';
    _.times(10, function (i) {
      ctx.moveTo(i * 10 + 0.5, 0.5);
      ctx.lineTo(i * 10 + 0.5, 100.5);
      ctx.moveTo(0.5, i * 10 + 0.5);
      ctx.lineTo(100.5, i * 10 + 0.5);
    });
    ctx.stroke();
    ctx.beginPath();
    ctx.strokeStyle = '#555';
    ctx.rect(0.5, 0.5, 100.5, 100.5);
    ctx.stroke();
    $('#grid').css('background-image', 'url(' + canvas.toDataURL('image/png') + ')');
  }($('#grid-img')[0]));

  (function (conv) {
    // JSONファイルを読み込むイベントの設定
    if ($window.File && $window.FileReader && $window.FileList && $window.JSON) {
      $('#file').change(function (e) {
        var fr = new FileReader();
        $scope.$apply(function () {
          $scope.isNowLoading = true;
        });
        fr.onload = (function () {
          return function (e) {
            var scope = angular.element('#viewer-controller').scope(),
              json;
            try {
              json = $.parseJSON(e.target.result);
            } catch (ee) {
              alert('正しいJSONファイルではありません');
            }
            if (json) {
              scope.$apply(function () {
                conv(json);
              });
            }
            $scope.$apply(function () {
              $scope.isNowLoading = false;
            });
          };
        }(e.target.files[0]));
        fr.readAsText(e.target.files[0]);
      });
    } else {
      alert('最新のブラウザを使用してください');
    }
    // URLでjsonファイルを指定している場合は、そのファイルを読み込む
    if ($location.search().data) {
      $scope.isNowLoading = true;
      $.getJSON($location.search().data, function (json) {
        $scope.$apply(function () {
          conv(json);
          $scope.isNowLoading = false;
        });
      });
    }
  }(function (json) {
    // JSONデータをオブジェクトに変換
    $scope.ninjaRadius = json[0][0];
    $scope.laps = json[0][1];
    $scope.stages = _.map(json[1], function (stageJson) {
      return {
        fieldW: stageJson[0][0],
        fieldH: stageJson[0][1],
        flowSpeed: stageJson[0][4],
        lotuses: _.map(stageJson[0][2], function (lotusJson) {
          return {
            x: lotusJson[0],
            y: lotusJson[1],
            radius: lotusJson[2]
          };
        }),
        rank: stageJson[0][3],
        score: stageJson[0][5],
        turns: _.map(stageJson[1], function (turnJson) {
          return {
            ninjas: _.map(turnJson[0], function (ninja) {
              return {
                x: ninja[0],
                y: ninja[1],
                accelCount: ninja[2],
                lotusCount: ninja[3],
                progress: ninja[3] * 100 / (stageJson[0][2].length * $scope.laps)
              };
            })
          };
        })
      };
    });
    $scope.currentStageNo = new Number(0);
  }));
  $scope.set = function (stageNo) {
    $scope.currentStageNo = stageNo;
  };
  (function (round) {
    // ステージ番号を範囲内に収める
    $scope.validateStage = function () {
      if (isNaN($scope.currentStageNo)) {
        $scope.currentStageNo = 0;
      }
      $scope.currentStageNo = round($scope.currentStageNo, $scope.stages);
    };
    // ターン番号を範囲内に収める
    $scope.validateTurn = function () {
      if (isNaN($scope.currentTurnNo)) {
        $scope.currentTurnNo = 0;
      }
      $scope.currentTurnNo = round($scope.currentTurnNo, $scope.currentStage.turns);
    };
  }(function (v, arr) {
    return Math.min(Math.max(v, 0), arr.length - 1);
  }));
  // ステージ番号に変化があったら、ステージ初期化
  $scope.$watch('currentStageNo', function (currentStageNo) {
    var width,
      height,
      ctx;
    $scope.currentTurnNo = new Number(0);

    if ($scope.stages) {
      $scope.currentStage = $scope.stages[currentStageNo];
      $slider.slider({
        min: 0,
        max: $scope.currentStage.turns.length - 1,
        step: 1,
        slide: function (e, ui) {
          $scope.$apply(function () { $scope.currentTurnNo = ui.value; });
        }
      });
      width = $scope.currentStage.fieldW;
      height = $scope.currentStage.fieldH;

      $('.screen').attr({width: width * 10, height: height * 10});
      ctx = $('#grid')[0].getContext('2d');
      ctx.clearRect(0, 0, width * 10, height * 10);

      // 蓮描画
      ctx.lineWidth = 1;
      ctx.strokeStyle = '#000';
      ctx.fillStyle = 'rgba(64, 255, 0, 0.5)';
      _.each($scope.currentStage.lotuses, function (lotus) {
        ctx.beginPath();
        ctx.arc(lotus.x * 10, lotus.y * 10, lotus.radius * 10, 0, Math.PI * 2, false);
        ctx.fill();
        ctx.stroke();
      });
      // 経路描画
      ctx = $('#pond-info')[0].getContext('2d');
      ctx.clearRect(0, 0, width * 10, height * 10);
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(255, 0, 0, 0.4)';
      ctx.beginPath();
      _.each($scope.currentStage.lotuses, function (lotus) {
        ctx.lineTo(lotus.x * 10 + 0.5, lotus.y * 10 + 0.5);
      });
      ctx.closePath();
      ctx.stroke();
      // 蓮番号表示
      ctx.font = '12px monospace';
      ctx.textAlign = 'center';
      _.each($scope.currentStage.lotuses, function (lotus, i) {
        var r = lotus.radius * 10 / 1.4142;
        ctx.beginPath();
        ctx.arc(lotus.x * 10 - r, lotus.y * 10 - r - 4, 9, 0, Math.PI * 2, false);
        ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
        ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.fillText(i, lotus.x * 10 - r, lotus.y * 10 - r);
      });

      // プログレスバーデータ作成
      $scope.progresses = _.map($scope.currentStage.rank, function () {
        return {
          result: 0,
          lotusCount: 0
        };
      });
    }
  });
  // ターン番号に変化があった場合の処理
  $scope.$watch('currentTurnNo', function (currentTurnNo) {
    if (!$scope.stages) {
      return;
    }
    $scope.currentTurn = $scope.currentStage.turns[currentTurnNo];
    var turnsInfo = $scope.currentStage.turns,
      $screen = $('#ninja'),
      ctx = $screen[0].getContext('2d'),
      width = $scope.currentStage.fieldW,
      height = $scope.currentStage.fieldH;
    ctx.clearRect(0, 0, width * 10, height * 10);
    // 忍者の残像描画
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.lineCap = ctx.lineJoin = 'round';
    ctx.lineWidth = $scope.ninjaRadius * 20;
    _.times(turnsInfo[0].ninjas.length, function (i) {
      var start = $scope.currentTurn.ninjas[i];
      _.times(10, function (j) {
        ctx.beginPath();
        ctx.moveTo(start.x * 10, start.y * 10);
        _.times(Math.min(j, currentTurnNo), function (k) {
          var ninja = turnsInfo[currentTurnNo - k].ninjas[i];
          ctx.lineTo(ninja.x * 10, ninja.y * 10);
        });
        ctx.stroke();
      });
    });

    // 忍者描画
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#000';
    _.each($scope.currentTurn.ninjas, function (ninja, i) {
      ctx.beginPath();
      $scope.progresses[i].lotusCount = ninja.lotusCount;
      $scope.progresses[i].result = ninja.progress;
      ctx.fillStyle = $scope.ninjaColor(i);
      ctx.arc(ninja.x * 10, ninja.y * 10, $scope.ninjaRadius * 10, 0, Math.PI * 2, false);
      ctx.fill();
      ctx.stroke();
    });

    // 残り加速数を表示
    ctx = $('#ninja-info')[0].getContext('2d');
    ctx.clearRect(0, 0, width * 10, height * 10);
    ctx.font = '12px monospace';
    ctx.textAlign = 'center';
    ctx.fillStyle = '#fff';
    _.each($scope.currentTurn.ninjas, function (ninja) {
      ctx.fillText(ninja.accelCount, ninja.x * 10, ninja.y * 10 + 4);
    });

    // 流れ画像のオフセット設定
    $('#pond').css('background-position', '0 ' + parseInt(currentTurnNo * $scope.currentStage.flowSpeed * 10, 10) + 'px');

    $slider.slider('option', {value: currentTurnNo});
  });
  // 再生ボタンが押された場合の処理
  (function () {
    var animHandle;
    $scope.play = function () {
      $scope.isPlay = !$scope.isPlay;
      if ($scope.isPlay) {
        (function loop() {
          $scope.currentTurnNo += 1;
          if ($scope.currentTurnNo >= $scope.currentStage.turns.length - 1) {
            $scope.isPlay = false;
            $timeout.cancel(animHandle);
          } else {
            animHandle = $timeout(function () {
              $scope.$apply(loop());
            }, $scope.wait);
          }
        }());
      } else {
        $timeout.cancel(animHandle);
      }
    };
  }());
  // Informationボタンが押されたら情報を見せる
  $scope.$watch('isShowInfo', function (isShowInfo) {
    $('#pond-info,#ninja-info').toggle(isShowInfo);
    $scope.currentTurnNo = new Number($scope.currentTurnNo);
  });
  // 再生状態をスライダーに反映させる
  $scope.$watch('isPlay', function (isPlay) {
    $slider.slider('option', {disabled: isPlay});
  });
}).filter('rank', function () {
  'use strict';
  var str = ['1st', '2nd', '3rd', '4th'];
  return function (rank) {
    return str[rank];
  };
});
