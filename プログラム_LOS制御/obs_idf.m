%{
    障害物センサで取得した値が他のクワッドロータであるかを識別する関数
%}

function obstacle_flag = obs_idf(ranges1,ranges2,QuadAng,stepnum,Q,loop,Center_num,Quad_num,obstacle_flag,lg)

    %空行列であれば0を代入している
    if(isempty(ranges1)==1)
        ranges1 = zeros(1,stepnum);
    end    
    if(isempty(ranges2)==1)
        ranges2 = zeros(1,stepnum);    
    end

    %センサデータの値が4より小さい要素のインデックスを取得
    index1 = find(ranges1 < 4);
    index2 = find(ranges2 < 4);
 
    %センサデータの値が4であれば障害物は存在しないため除外
    ranges1 = ranges1(ranges1 < 4);
    ranges2 = ranges2(ranges2 < 4);

    %検知された障害物がフォロワーかの判定に使用
    Quadindex = 1:5;
    Quadindex = Quadindex(Quadindex ~= Center_num);
    
    %センサ1で障害物を検知した場合     
    if ~isempty(ranges1)     
        %抽出したデータに対してループを回す    
        for j  = 1: length(ranges1)     
            theta1 = 4/3*pi/stepnum * (index1(j)-1) - pi*1/6 -pi/2 + QuadAng(3);
            %ステップの角度にセンサの距離をかけて座標を求める 
            x = ranges1(j)*cos(theta1) + Q.Coord(1,loop,Center_num)/100 + 0.1;            
            y = ranges1(j)*sin(theta1) + Q.Coord(2,loop,Center_num)/100;
            obs_Coord = [x*100,y*100];
            obs_Coord = obs_Coord.';
            obsflag = 1;
            for k = 1:Quad_num-1
                %フォロワーとの実座標の距離で分岐   
                if norm(obs_Coord-Q.Coord(1:2,loop,Quadindex(k))) > 100
                    obsflag = obsflag & 1;
                else
                    obsflag = obsflag & 0;
                end
            end
            if norm(obs_Coord-lg(1:2)) > 100
                obsflag = obsflag & 1;
            else
                obsflag = obsflag & 0; 
            end
            ranges1(j) = obsflag;
         end

        if sum(ranges1) > 0 
           obstacle_flag = 1;
        else
            obstacle_flag = 0;
        end
    else
        obstacle_flag = 0;
    end
    %センサ2で障害物を検知した場合    
    if  ~isempty(ranges2) 
        for j = 1:length(ranges2)
            theta2 = 4/3*pi/stepnum * (index2(j)-1) - pi*1/6 + pi/2 + QuadAng(3);
            x = ranges2(j)*cos(theta2) +  Q.Coord(1,loop,Center_num)/100 -0.1;
            y = ranges2(j)*sin(theta2) +  Q.Coord(2,loop,Center_num)/100;
            obs_Coord = [x*100,y*100];
            obs_Coord = obs_Coord.';
            obsflag = 1;
            for k = 1:Quad_num-1
                if norm(obs_Coord-Q.Coord(1:2,loop,Quadindex(k))) > 100
                    obsflag = obsflag & 1;   
                else
                    obsflag = obsflag & 0;  
                end
            end
            %目標地点の識別
            if norm(obs_Coord-lg(1:2)) > 100
                obsflag = obsflag & 1;
            else
                obsflag = obsflag & 0; 
            end
            ranges2(j) = obsflag;
        end
            
        if sum(ranges2) > 0 
            obstacle_flag = obstacle_flag || 1;
        else
            obstacle_flag = obstacle_flag || 0;
        end
    else
        obstacle_flag = 0;
    end
end