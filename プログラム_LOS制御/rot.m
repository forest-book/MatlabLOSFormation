%{
    回転行列を返す関数
    ロドリゲスの回転公式を使用
%}

function r=rot(x,y,z,th)
I = eye(3);             % 3x3の単位行列
% 直積行列
kron=[x^2,x*y,x*z;x*y,y^2,y*z;x*z,y*z,z^2];
% クロス積行列
Crossprd=[0, -z, y;z,0, -x;-y,x,0];
r=cos(th)*I+sin(th)*Crossprd+(1-cos(th))*kron;
end