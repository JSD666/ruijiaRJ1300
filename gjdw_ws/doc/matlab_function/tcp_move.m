function distance=tcp_move(var_INT_translate,AXE,RPY)
%LIDARTOMA_ID：是对应函数的名称
%var_INT_translate：TCP移动的长度单位mm
%AXE：TCP移动的方向
%RPY:对应函数求出来的RPY角度：[psai theta phi]
d=var_INT_translate;
psai=RPY(1);
theta=RPY(2);
phi=RPY(3);
switch AXE
    case '-z'
        tcp=[0; 0; -d;1];
    case '-y'
        tcp=[0; -d; 0;1];
    case '-x'
        tcp=[-d; 0; 0;1];
end
d=rot('z',phi)*rot('y',theta)*rot('x',psai)*tcp;
distance=d(1:3,1);
end