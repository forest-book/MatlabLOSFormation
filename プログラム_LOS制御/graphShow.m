
data = zeros(loop-1,4);

for i = 1:loop-1
    for j = 1:4
        data(i,j) = norm(l_Error(1:2,i,j))/100; 
    end
end

yoko = 1:loop-1;

color = ["black","black","black","black"];

for i = 1:4
    f(i) = figure;
    figure(f(i));
    plot(yoko,data(:,i),color(i));
    xlabel("ステップ数");
    ylabel("追従偏差[m]")
    xlim([0 loop])
    ylim([0 2])
end

%{
for i = 1:1092
    speed_data(i) = norm(Q.speed(:,i,5));
end
%}

f_l = figure;
figure(f_l);
plot(yoko(1:loop-1),Q.speed_dir(3,1:loop-1,5));

f_l1 = figure;
figure(f_l1);
plot(yoko(1:loop-1),Q.speed_dir(3,1:loop-1,1));


