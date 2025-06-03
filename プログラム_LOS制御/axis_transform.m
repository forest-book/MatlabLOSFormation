%{
    ワールド座標系でみたローカル座標軸を取得する関数
    引数：正規化されたリーダの速度
　　戻り値：ワールド座標系でみたローカル座標軸
    実装重視でプログラムの整理は行っていない
%}

function new_axis = axis_transform(speed_dir)

    new_axis = zeros(3,3);

   %元のプログラムに合わせるために変数名をvにして転置
   v = speed_dir(:).';
   
    % 元のxyz軸
    x_axis = [1, 0, 0];
    y_axis = [0, 1, 0];
    z_axis = [0, 0, 1];

    % 回転軸（元のy軸と新しいy'軸の外積）
    % 外積の結果として得られるベクトルは元の二つのベクトルに直交するので回転軸となる
    rotation_axis = cross(y_axis, v);

    % 回転角を計算（元のy軸と新しいy'軸の間の角度）
    cos_theta = dot(y_axis, v);
    sin_theta = norm(rotation_axis);
    theta = atan2(sin_theta, cos_theta);

    % 回転軸の正規化
    rotation_axis = rotation_axis / norm(rotation_axis);

    % Rodriguesの回転公式を使用して回転行列を計算
    K = [0, -rotation_axis(3), rotation_axis(2); 
        rotation_axis(3), 0, -rotation_axis(1); 
        -rotation_axis(2), rotation_axis(1), 0];
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K ^ 2);

    new_axis(:,1) = R*x_axis';
    new_axis(:,2) = R*y_axis';
    new_axis(:,3) = R*z_axis';
end

