%{
    本プログラムでは距離等の単位を[cm]で考える
%}
%% メモリのクリア
clear;
close all
clc

%% CoppeliaSimとの連携
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); %just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
pause(0.1)

%{ 
    clientID1:リーダ,clientID2以降:フォロワ1・2・3・4
    フォロワを増やす際にはコピペ
    19999の場所に当たる数字のみcoppeliasimと一致させる
%}
clientID1 = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
clientID2 = sim.simxStart('127.0.0.1', 19995, true, true, 5000, 5);
clientID3 = sim.simxStart('127.0.0.1', 19993, true, true, 5000, 5);
clientID4 = sim.simxStart('127.0.0.1', 19991, true, true, 5000, 5);
clientID5 = sim.simxStart('127.0.0.1', 19989, true, true, 5000, 5);
%リーダの目標点(円筒形)を表示する
clientID6 = sim.simxStart('127.0.0.1', 19987, true, true, 5000, 5);

%% coppeliasimとの連携
if(clientId > -1)
    disp('Connected to remote API server');

    %{
        handle setting
        Quadricopter_targetはcoppeliasimのドローンの球
    %}
    [r, Quad(1)]   = sim.simxGetObjectHandle(clientID1, 'Quadcopter', sim.simx_opmode_blocking);
    [r, target(1)] = sim.simxGetObjectHandle(clientID1, 'Quadcopter_target', sim.simx_opmode_blocking);

    [r, Quad(2)]   = sim.simxGetObjectHandle(clientID2, 'Quadcopter#1', sim.simx_opmode_blocking);
    [r, target(2)] = sim.simxGetObjectHandle(clientID2, 'Quadcopter_target#1', sim.simx_opmode_blocking);

    [r, Quad(3)]   = sim.simxGetObjectHandle(clientID2, 'Quadcopter#3', sim.simx_opmode_blocking);
    [r, target(3)] = sim.simxGetObjectHandle(clientID2, 'Quadcopter_target#3', sim.simx_opmode_blocking);

    [r, Quad(4)]   = sim.simxGetObjectHandle(clientID2, 'Quadcopter#5', sim.simx_opmode_blocking);
    [r, target(4)] = sim.simxGetObjectHandle(clientID2, 'Quadcopter_target#5', sim.simx_opmode_blocking);

    [r, Quad(5)]   = sim.simxGetObjectHandle(clientID2, 'Quadcopter#7', sim.simx_opmode_blocking);
    [r, target(5)] = sim.simxGetObjectHandle(clientID2, 'Quadcopter_target#7', sim.simx_opmode_blocking);

    [r, Cylinder]  = sim.simxGetObjectHandle(clientID6, 'Cylinder', sim.simx_opmode_blocking);

    % Get position and orientation data setting
    [returnCode, QuadPos1] = sim.simxGetObjectPosition(clientID1, Quad(1), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng1] = sim.simxGetObjectOrientation(clientID1, Quad(1), -1, sim.simx_opmode_streaming);
    pause(0.1);
    % Get URG sensor data
    % todo LiDARのデータ周りのコードを確かめる
    [errorCode, dist] = sim.simxGetStringSignal(clientID1, 'scan ranges11', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID1, 'scan ranges12', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos2] = sim.simxGetObjectPosition(clientID2, Quad(2), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng2] = sim.simxGetObjectOrientation(clientID2, Quad(2), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID2, 'scan ranges21', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID2, 'scan ranges22', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos3] = sim.simxGetObjectPosition(clientID3, Quad(3), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng3] = sim.simxGetObjectOrientation(clientID3, Quad(3), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID3, 'scan ranges31', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID3, 'scan ranges32', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos4] = sim.simxGetObjectPosition(clientID4, Quad(4), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng4] = sim.simxGetObjectOrientation(clientID4, Quad(4), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID4, 'scan ranges41', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID4, 'scan ranges42', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos5] = sim.simxGetObjectPosition(clientID5, Quad(5), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng5] = sim.simxGetObjectOrientation(clientID5, Quad(5), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID5, 'scan ranges51', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID5, 'scan ranges52', sim.simx_opmode_streaming);
    pause(0.1);

    %% フォーメーション制御(以下の部分が処理のメイン)
    
    %% 変数の規定(シミュレーション状況により変更する)

    %ステップ数(シミュレーション時間)
    loop_num = 20000;

    %リーダ機の機体番号を格納(値は簡易的に1としている)
    Lnumber = 1;

    %リーダを含めたクワッドローターの数
    Quad_num = 5;

    %距離の閾値(侵入禁止領域の設定)
    thresho = 80;

    %更新ステップ幅,Δt
    dt = 1;

    %k0lはフォロワの目標速度(制御入力)の方向を計算する際のゲイン(k0,kl)
    k01 = zeros(Quad_num-1, 2);
    k01(1, :) = [5, 200];
    k01(2, :) = [5, 200];
    k01(3, :) = [5, 200];
    k01(4, :) = [5, 200];

    %kpsはフォロワの目標速度(制御入力)の大きさを計算する際のゲイン(kp,ks)
    kps = zeros(Quad_num-1, 2);
    kps(1, :) = [1, 1];
    kps(2, :) = [1, 1];
    kps(3, :) = [1, 1];
    kps(4, :) = [1, 1];

    %フォーメーションの数
    formation_num = 2;

    %フォーメーションを指定
    current_formation = 2;

    %リーダと各フォロワの距離を規定 d = distanse
    d = zeros(Quad_num-1, 2);
    d(:, 1) = [320, 240, 160, 80]; %直線フォーメーション
    d(:, 2) = [200, 200, 100, 100]; %くの字フォーメーション

    %リーダの目標到達地点
    lg = zeros(3,LG_num);

    %lg(:,1) = [500,-15,250]; %これは何の設定？
    lg(:,1) = [1100,-15,250];
    lg(:,2) = [-650,-15,300];

    %% クワッドローターの構造体の定義と定数の規定

    %{
        この構造体では機体番号(属性番号で無い)に紐づく情報を保持している
        機体番号はインデックスが対応する
        各クワッドローターの情報を格納する構造体の定義
        Q:構造体名
        Att:属性番号(機体番号に対応する属性番号を格納)
            リーダ：1
            2以降:フォロワ
                    5~2がそれぞれフォロワ1~4に対応
        Coord:座標
                第1インデックス:x,y,z
                第2インデックス:ステップ
                第3インデックス:機体番号
        l_distance:自身から見たリーダのベクトル
                        インデックスは座標と同じ
        unit_l_distance:l_distanceの単位ベクトルを格納
                        インデックスは座標と同じ
        speed:速さ,インデックスは座標と同じ
        speed_dir:速度の方向,インデックスは座標と同じ
        Cin:制御入力の大きさ
            第1インデックス:ステップ
            第2インデックス:機体番号
        Cin_dir:Cinの単位ベクトル
                第1インデックス:x,y,z
                第2インデックス:ステップ
                第3インデックス:機体番号
    %}




