%{
    リーダーの変更を行う関数    
    gd:1列目は目標点との距離を格納
      :2列目は属性番号を格納    
%}
function R=ChangeLeader(Q,loop,Quad_num,lg,Change_num)
    gd = zeros(Quad_num,2);        
    
    %クワッドローターと目標地点の距離と属性番号格納
    for t = 1:Quad_num        
        gd(t,1) = norm(lg(:,Change_num)-Q.Coord(:,loop,t));        
        gd(t,2) = Q.Att(t);        
    end        

    %目標点との距離が大きい順にソート      
    for t = 1:Quad_num-1         
        for s = 1:Quad_num-1        
            if gd(s,1) < gd(s+1,1)        
                tmp1 = gd(s,1);        
                tmp2 = gd(s,2);        
                gd(s,:) = gd(s+1,:);        
                gd(s+1,1) = tmp1;        
                gd(s+1,2) = tmp2;        
            end        
        end        
    end

    R = zeros(Quad_num,1);  
    
    %属性番号を再決定する
    for s = 1:Quad_num-1        
        R(Q.Att == gd(s,2)) = s+1;        
    end        
    R(Q.Att==gd(Quad_num,2)) = 1;   
    
end