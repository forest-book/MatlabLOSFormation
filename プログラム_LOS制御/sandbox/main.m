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
    [errorCode, dist] = sim.simxGetStringSignal(clientID1, 'scan_ranges11', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID1, 'scan_ranges12', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos2] = sim.simxGetObjectPosition(clientID2, Quad(2), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng2] = sim.simxGetObjectOrientation(clientID2, Quad(2), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID2, 'scan_ranges21', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID2, 'scan_ranges22', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos3] = sim.simxGetObjectPosition(clientID3, Quad(3), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng3] = sim.simxGetObjectOrientation(clientID3, Quad(3), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID3, 'scan_ranges31', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID3, 'scan_ranges32', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos4] = sim.simxGetObjectPosition(clientID4, Quad(4), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng4] = sim.simxGetObjectOrientation(clientID4, Quad(4), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID4, 'scan_ranges41', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID4, 'scan_ranges42', sim.simx_opmode_streaming);
    pause(0.1);

    [returnCode, QuadPos5] = sim.simxGetObjectPosition(clientID5, Quad(5), -1, sim.simx_opmode_streaming);
    [returnCode, QuadAng5] = sim.simxGetObjectOrientation(clientID5, Quad(5), -1, sim.simx_opmode_streaming);
    pause(0.1);
    [errorCode, dist] = sim.simxGetStringSignal(clientID5, 'scan_ranges51', sim.simx_opmode_streaming);
    [errorCode, dist] = sim.simxGetStringSignal(clientID5, 'scan_ranges52', sim.simx_opmode_streaming);
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
    Q.Att = zeros(Quad_num, 1);
    Q.Coord = zeros(3, loop_num+1, Quad_num);
    Q.l_distance = zeros(3, loop_num+1, Quad_num);
    Q.unit_l_distance = zeros(3, loop_num+1, Quad_num);
    Q.speed = zeros(3, loop_num+1, Quad_num);
    Q.speed_dir = zeros(3, loop_num+1, Quad_num);
    Q.Cin = zeros(3, loop_num+1, Quad_num);
    Q.Cin_dir = zeros(3, loop_num+1, Quad_num);

    %{
        フォロワ間のベクトルを格納する配列
        フォロワ番号(属性番号の2以降に紐づく情報)
        VBF(vector between followers):フォロワ間のベクトル
        VBF_dir:VBFの単位ベクトル
    %}
    VBF = zeros(3, loop_num, Quad_num-1, Quad_num-1);
    VBF_dir = zeros(3, loop_num, Quad_num-1, Quad_num-1);

    %{
        目標点に到達した時間を格納する配列
        arrive_timeに格納された時間を用いて目標点を表示する
    %}
    arrive_time = zeros(LG_num, 1);

    %% 初期設定

    %属性番号の初期化
    Q.Att = 1:5;

    %{
        AOR(Angle of rotation):フォロワの追従位置を決定する角度
        一列目がΨ,二列目がΦ
        ψがローカルでのz軸中心の回転 θがローカルでのx軸周りの回転
    %}
    AOR = zeros(Quad_num-1, 2, formation_num);
    % 縦列フォーメーションの追従角度
    AOR(1,:,1) = [deg2rad(180),deg2rad(0)];   %フォロワ1の回転角ΨとΦ
    AOR(2,:,1) = [deg2rad(180),deg2rad(0)];   %フォロワ2の回転角ΨとΦ 
    AOR(3,:,1) = [deg2rad(180),deg2rad(0)];   %フォロワ3の回転角ΨとΦ
    AOR(4,:,1) = [deg2rad(180),deg2rad(0)];   %フォロワ4の回転角ΨとΦ

    % くの字フォーメーションの追従角度
    AOR(1,:,2) = [deg2rad(-140), deg2rad(0)];   %フォロワ1の回転角ΨとΦ
    AOR(2,:,2) = [deg2rad(140),  deg2rad(0)];   %フォロワ2の回転角ΨとΦ 
    AOR(3,:,2) = [deg2rad(140),  deg2rad(0)];   %フォロワ3の回転角ΨとΦ
    AOR(4,:,2) = [deg2rad(-140), deg2rad(0)];   %フォロワ4の回転角ΨとΦ

    %リーダの初期座標を設定
    Q.Coord(1, 1, 1) = -400;
    Q.Coord(2, 1, 1) = 0;
    Q.Coord(3, 1, 1) = 220;

    %フォロワ1,2,3,4の初期座標を設定
    Q.Coord(1, 1, 2) = -420; Q.Coord(2, 1, 2) = -110; Q.Coord(3, 1, 2) = 250;   %フォロワ1
    Q.Coord(1, 1, 3) = -500; Q.Coord(2, 1, 3) = -60;  Q.Coord(3, 1, 3) = 250;   %フォロワ2
    Q.Coord(1, 1, 4) = -520; Q.Coord(2, 1, 4) =  45;  Q.Coord(3, 1, 4) = 250;   %フォロワ3
    Q.Coord(1, 1, 5) = -600; Q.Coord(2, 1, 5) = -110; Q.Coord(3, 1, 5) = 250;   %フォロワ4

    %リーダの目標地点の変更回数
    Change_num = 1;

    %全目標地点に到達した時点で停滞させるための座標
    Comp_Coord = zeros(3, Quad_num);

    %上記座標を取得するためのフラグ
    Comp_flag = 0;

    %例外に到達したことを示すフラグ
    exception_flag = 0;

    %例外時に停滞させるための座標
    exception_Coord = zeros(3, Quad_num);

    %障害物センサ配列の要素数
    stepnum = 684;

    %障害物があるかの判定
    obstacle_flag = 0;

    %グラフ出力用の配列
    l_Error = zeros(3, loop_num, 4);

    %ワールド座標系でみたローカル軸
    new_aixs = zeros(3, 3);

    %クワッドロータの初期座標をCoppeliaSimに反映している
    sim.simxSetObjectPosition(clientID1, Quad(1), -1, [Q.Coord(1,1,1)/100, Q.Coord(2,1,1)/100, Q.Coord(3,1,1)/100], sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID2, Quad(2), -1, [Q.Coord(1,1,2)/100, Q.Coord(2,1,2)/100, Q.Coord(3,1,2)/100], sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID3, Quad(3), -1, [Q.Coord(1,1,3)/100, Q.Coord(2,1,3)/100, Q.Coord(3,1,3)/100], sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID4, Quad(4), -1, [Q.Coord(1,1,4)/100, Q.Coord(2,1,4)/100, Q.Coord(3,1,4)/100], sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID5, Quad(5), -1, [Q.Coord(1,1,5)/100, Q.Coord(2,1,5)/100, Q.Coord(3,1,5)/100], sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID6, Cylinder, -1, [lg(1,1)/100, lg(2,1)/100, lg(3,1)/100], sim.simx_opmode_oneshot);
    sim.simxSynchronousTrigger(clientID);

    %ステップ分ループを回す
    for loop = 1:loop_num

        %% 実座標の取得
        [returnCode, QuadPos1] = sim.simxGetObjectPosition(clientID1, Quad(1), -1, sim.simx_opmode_buffer);
        [returnCode, QuadPos2] = sim.simxGetObjectPosition(clientID2, Quad(2), -1, sim.simx_opmode_buffer);
        [returnCode, QuadPos3] = sim.simxGetObjectPosition(clientID3, Quad(3), -1, sim.simx_opmode_buffer);
        [returnCode, QuadPos4] = sim.simxGetObjectPosition(clientID4, Quad(4), -1, sim.simx_opmode_buffer);
        [returnCode, QuadPos5] = sim.simxGetObjectPosition(clientID5, Quad(5), -1, sim.simx_opmode_buffer);

        Q.Coord(:, loop, 1) = QuadPos1(:) * 100;    
        Q.Coord(:, loop, 2) = QuadPos2(:) * 100;
        Q.Coord(:, loop, 3) = QuadPos3(:) * 100;   
        Q.Coord(:, loop, 4) = QuadPos4(:) * 100;
        Q.Coord(:, loop, 5) = QuadPos5(:) * 100;

        %% 初期状態でのリーダとフォロワの決定
        if loop == 1
            Q.Att(:) = ChangeLeader(Q, loop, Quad_num, lg, Change_num);
        end

        %% リーダ属性のクワッドロータの機体番号を取得
        Lnumber = find(Q.Att == 1);

        %% 障害物センサの処理
        %{
            ・基本的にはリーダの障害物センサの値のみを利用する
            ・フォーメーションが1でリーダが障害物を検知しなければ最後尾の
　　　　　　　クワッドロータの障害物センサの値を利用する
            ・センサの最大値は4であり,4のデータは障害物が存在しない
            ・フォロワと目標点も障害物センサに反応するため除外する
        %}
        %% 通常の障害物検知

        %リーダ機の障害物センサの値を取得する  
        [errorCode, tmp_ranges1] = sim.simxGetStringSignal(Lnumber, ['scan_ranges' num2str(Lnumber) '1'], sim.simx_opmode_buffer);
        [errorCode, tmp_ranges2] = sim.simxGetStringSignal(Lnumber, ['scan_ranges' num2str(Lnumber) '2'], sim.simx_opmode_buffer);
        %数値データに変換
        ranges1 = sim.simxUnpackFloats(tmp_ranges1);
        ranges2 = sim.simxUnpackFloats(tmp_ranges2);
        %リーダ機の姿勢の取得
        [returnCode, QuadAng] = sim.simxGetObjectOrientation(find(Q.Att == 1), Quad(Q.Att == 1), -1, sim.simx_opmode_buffer);   
        pause(0.05);
        %障害物があるかの判定
        obstacle_flag = obs_idf(ranges1, ranges2, QuadAng, stepnum, Q, loop, Lnumber, Quad_num, obstacle_flag, lg(1:2, Change_num));

        %% チョークポイントを抜けたかの判定
        if obstacle_flag == 0 && current_formation == 1
            %最後尾のクワッドロータの障害物センサの値を取得
            [errorCode, tmp_ranges1] = sim.simxGetStringSignal(find(Q.Att == 2), ['scan_ranges' num2str(find(Q.Att == 2)) '1'], sim.simx_opmode_buffer);
            [errorCode, tmp_ranges2] = sim.simxGetStringSignal(find(Q.Att == 2), ['scan_ranges' num2str(find(Q.Att == 2)) '2'], sim.simx_opmode_buffer);
            %数値データに変換
            ranges1 = sim.simxUnpackFloats(tmp_ranges1);
            ranges2 = sim.simxUnpackFloats(tmp_ranges2);
            %最後尾のクワッドロータの姿勢を取得
            [returnCode, QuadAng] = sim.simxGetObjectOrientation(find(Q.Att == 2), Quad(Q.Att == 2), -1, sim.simx_opmode_buffer);   
            pause(0.05);
            %障害物があるかの判定
            obstacle_flag = obs_idf(ranges1, ranges2, QuadAng, stepnum, Q, loop, find(Q.Att == 2), Quad_num, obstacle_flag, lg(1:2, Change_num));
        end

        %障害物の有無によりフォーメーションを指定
        if obstacle_flag == 1
            %フォーメーションを指定(一直線の形)
            current_formation = 1;
        else
            %フォーメーションを指定(くの字の形)
            current_formation = 2;
        end

        %% リーダの処理
        %{   
            リーダ機の速さ及び単位ベクトルの格納
            リーダ機の次ステップにおける座標の計算を行う
        %}

        %リーダから見た目標地点のベクトル
        G_l = lg(:, Change_num) - Q.Coord(:, loop, Lnumber);
        %リーダの速度を算出,計算の仕方は単位ベクトル×スカラーの速さ
        Q.speed(:, loop, Lnumber) = G_l / norm(G_l) * L_speed;
        Q.speed_dir(:, loop, Lnumber) = Q.speed(:, loop, Lnumber) / norm(Q.speed(:, loop, Lnumber));

        %リーダの次ステップでの座標を算出
        Q.Coord(:, loop+1, Lnumber) = Q.Coord(:, loop, Lnumber) + dt * Q.speed(:, loop, Lnumber);

        %ゼロ除算回避で0を直接代入している
        Q.l_distance(:, loop, Lnumber) = 0;       
        Q.unit_l_distance(1:2, loop, Lnumber) = 0;

        %% フォロワの処理

        %通常動作の処理
        if norm(G_l) > 30 %なぜ30?
            %{
                フォロワから見たリーダのベクトル及び単位ベクトルを算出
                属性番号がリーダである機体はゼロ除算を回避するために分岐させる
            %}
            for i = 1:Quad_num-1
                Q.l_distance(:, loop, find(Q.Att == i+1)) = Q.Coord(:, loop, Lnumber) - Q.Coord(:, loop, find(Q.Att == i+1));   
                Q.unit_l_distance(1:2, loop, find(Q.Att == i+1)) = Q.l_distance(1:2, loop, find(Q.Att == i+1)) / norm(Q.l_distance(1:2, loop, find(Q.Att == i+1)));
            end

            %{
                フォロワ間のベクトルを算出(接近禁止領域の規定)
                属性番号に紐づける
            %}   
            for i = 1:Quad_num-1
                for j = i+1:Quad_num-1
                    VBF(:, loop, j, i) = Q.Coord(:, loop, find(Q.Att == i+1)) - Q.Coord(:, loop, find(Q.Att == j+1));
                    VBF_dir(1:2, loop, j, i) = VBF(1:2, loop, j, i) / norm(VBF(1:2, loop, j, i));
                end
            end

            %% フォロワ番号により条件確認の範囲が異なる処理
        
            %{
                フォロワ番号でループを回す
                フォロワが条件を満たすかの判定はflagで行う
            %}
            for i = 1:Quad_num-1
                flag = 1;
                %フォロワ単体に関する処理は以下のforループと同じインデントで行う

                %他のフォロワとの距離が適切かの判定
                for j = i+1:Quad_num-1
                    if norm(VBF(1:2, loop, j, i)) >= thresho
                        flag = flag & 1;
                    else
                        flag = flag & 0;
                        break
                    end
                end

                %リーダとの距離が適切かの判定
                if norm(Q.l_distance(1:2,loop,find(Q.Att == i+1))) >= thresho
                    flag = flag & 1;     
                else     
                    flag = flag & 0;     
                end

                %%記録用
                D_1 = d(i,current_formation) * rot(new_aixs(1,1), new_aixs(2,1), new_aixs(3,1), AOR(i, 2, current_formation)) * rot(new_aixs(1,3), new_aixs(2,3), new_aixs(3,3), AOR(i, 1, current_formation)) * Q.speed_dir(:, loop, Lnumber);    
                %フォロワからみた目標地点へのベクトル   
                l_Error(:, loop, i) = Q.Coord(:, loop, Lnumber) - Q.Coord(:, loop, find(Q.Att == i+1)) + D_1; 

                if flag == 1 
                    %回転ベクトル
                    D = d(i,current_formation) * rot(new_aixs(1,1), new_aixs(2,1), new_aixs(3,1), AOR(i, 2, current_formation)) * rot(new_aixs(1,3), new_aixs(2,3), new_aixs(3,3), AOR(i, 1, current_formation)) * Q.speed_dir(:, loop, Lnumber);             
                    %フォロワからみた目標地点へのベクトル   
                    lt = Q.Coord(:, loop, Lnumber) - Q.Coord(:, loop, find(Q.Att == i+1)) + D;   
                    
                    %式(3.13)    
                    h = k0l(i,1) + k0l(i,2) / (1+norm(lt));    
                    %式(3.11)    
                    Q.Cin_dir(:, loop, find(Q.Att == i+1)) = (lt + h * Q.speed_dir(:,loop,Lnumber)) / norm(lt + h * Q.speed_dir(:, loop, Lnumber));    
                    %式(3.14)    
                    Q.Cin(loop, find(Q.Att == i+1)) = norm(Q.speed(:, loop, Lnumber)) * (1 + (2/pi) * kps(i,1) * atan(abs(dot(lt, Q.speed_dir(:, loop, Lnumber)) / kps(i,2))));    
                    %最終的な速度の計算
                    Q.speed(:, loop, find(Q.Att == i+1)) = Q.Cin(loop, find(Q.Att == i+1)) * Q.Cin_dir(:, loop, find(Q.Att == i+1));
                end

                %リーダとフォロワの回避関係
                Q.unit_l_distance(1:2, loop, find(Q.Att == i+1)) = Q.l_distance(1:2, loop, find(Q.Att == i+1)) / norm(Q.l_distance(1:2, loop, find(Q.Att == i+1)));
       
                %リーダに近づきすぎた時の処理
                if norm(Q.l_distance(1:2, loop, find(Q.Att == i+1))) < thresho
                    %normの前の-1はdistanceをどちらを起点にするかによって必要か変わる 
                    Q.speed(1:2, loop, find(Q.Att == i+1)) = -1 * norm(Q.speed(1:2, loop, Lnumber)) * Q.unit_l_distance(1:2, loop, find(Q.Att == i+1)); %normの前の-1はdistanceをどっちを起点にするかによって必要か変わる
                    Q.speed(3, loop, find(Q.Att == i+1)) = 0;
                end
            end

            %% フォロワ同士の回避アルゴリズム(若い番号が回避するとする) 
            %他のフォロワとの距離が適正であるかの判別,フォロワ番号でループを回す
            for j = 1:Quad_num-1    
                for k = j+1:Quad_num-1            
                    if norm(VBF(1:2, loop, k, j)) < thresho              
                        Q.speed(1:2, loop, find(Q.Att == j+1)) = norm(Q.speed(1:2, loop, find(Q.Att == k+1))) * VBF_dir(1:2, loop, k, j);
                        Q.speed(3, loop, find(Q.Att == j+1)) = 0;    
                    end
                end
            end

            %% 各機体の次の座標の計算と各機体が一直線に並んでいる状態でリーダ時の処理(折り返しのアルゴリズム)
            %フォロワ番号でループ
            for k = 1:Quad_num-1        
                %全フォロワの次の座標を算出
                Q.Coord(:, loop+1, find(Q.Att == k+1)) = Q.Coord(:, loop, find(Q.Att == k+1)) + dt * Q.speed(:, loop, find(Q.Att == k+1));
            end

        %%目標地点に到達した時の処理
        elseif norm(G_l) <= 30

            %目標地点到達した時間を記録(目標地点を表示する際に使用)
            arrive_time(Change_num, 1) = loop;
            %次の目標地点移る
            Change_num = Change_num + 1;
            %フォーメーションの指定
            current_formation = 2;

            %{
                分岐1:全目標地点に到達
                      最初に全目標地点に到達した時の座標を保存する(停留のため)
                分岐2:全目標地点に到達していない
                      属性を再定義し,1ステップ停留させる
                      次の目標地点に向かう
            %}
            if Change_num >= LG_num+1
                %停留座標の取得
                if Comp_flag == 0   
                    for i = 1:Quad_num
                        Comp_Coord(:, i) = Q.Coord(:, loop, i);
                    end
                end
                Comp_flag = 1;
                Change_num = Change_num-1;
            else
                Q.Att(:) = ChangeLeader(Q, loop, Quad_num, lg, Change_num);
                for i = 1:Quad_num
                    Comp_Coord(:, i) = Q.Coord(:, loop, i);
                end
            end
            
            %次のステップでの座標
            for i = 1:Quad_num    
                Q.Coord(:, loop+1, i) = Comp_Coord(:, i);    
            end
        %例外が発生した場合には全クワッドロータの動きを止める
        else
            %最初に例外になった時の座標を保存する(停留のため)
            if exception_flag == 0
                 for k = 1:Quad_num  
                     exception_Coord(:, k) = Q.Coord(:, k);
                 end
                 exception_flag = 1;
            end

            %全クワッドローダーを停留させる
            for k = 1:Quad_num
                Q.speed(:, loop:loop_num, k) = zeros(3, loop_num-loop+1);
                Q.Coord(:, loop+1, k) = exception_Coord(:, k);
            end 
        end

        %CoppeliaSimへの反映
        sim.simxSetObjectPosition(clientID1, target(1), -1, [Q.Coord(1,loop+1,1)/100, Q.Coord(2,loop+1,1)/100,Q.Coord(3,loop+1,1)/100], sim.simx_opmode_oneshot);
        sim.simxSetObjectPosition(clientID2, target(2), -1, [Q.Coord(1,loop+1,2)/100, Q.Coord(2,loop+1,2)/100,Q.Coord(3,loop+1,2)/100], sim.simx_opmode_oneshot);
        sim.simxSetObjectPosition(clientID3, target(3), -1, [Q.Coord(1,loop+1,3)/100, Q.Coord(2,loop+1,3)/100,Q.Coord(3,loop+1,3)/100], sim.simx_opmode_oneshot);
        sim.simxSetObjectPosition(clientID4, target(4), -1, [Q.Coord(1,loop+1,4)/100, Q.Coord(2,loop+1,4)/100,Q.Coord(3,loop+1,4)/100], sim.simx_opmode_oneshot);
        sim.simxSetObjectPosition(clientID5, target(5), -1, [Q.Coord(1,loop+1,5)/100, Q.Coord(2,loop+1,5)/100,Q.Coord(3,loop+1,5)/100], sim.simx_opmode_oneshot);
        sim.simxSetObjectPosition(clientID6, Cylinder, -1,  [lg(1,Change_num)/100, lg(2,Change_num)/100,lg(3,Change_num)/100], sim.simx_opmode_oneshot);
        sim.simxSynchronousTrigger(clientID);
        pause(0.05);
    end
else
    disp('Failed connecting to remote API server');
end


