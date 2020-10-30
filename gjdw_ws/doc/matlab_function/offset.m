function offset=offset(direction,value,var_TCP_varname)
%var_TCP_varname:根据工具坐标系待改变的路点（x y z rx ry rz）欧拉角：度数(没有欧拉叫用C++包或者脚本转)
%value：移动距离，可有正负float
%direction:1/2/3/4/5/6分别代表x/y/z/rx/ry/rz
x=var_TCP_varname(1);
y=var_TCP_varname(2);
z=var_TCP_varname(3);
psai=var_TCP_varname(4);
theta=var_TCP_varname(5);
phi=var_TCP_varname(6);
switch direction
    case 3 %z轴方向平移
        tcp=[0; 0; value;1];
    case 2 %y轴方向平移
        tcp=[0; value; 0;1];
    case 1 %x轴方向平移
        tcp=[value; 0; 0;1];
        
        %%考虑运算环境是弧度还是角度。如果是弧度，先把value转成弧度在运算 degree=(rad*180)/pi
    case 4 %x轴方向旋转
        tcp=[value; 0; 0;1];
    case 5 %x轴方向旋转
        tcp=[0; value; 0;1];
    case 6 %x轴方向旋转
        tcp=[0; 0; value;1];
        
end

d=rot('z',phi)*rot('y',theta)*rot('x',psai)*tcp;

if direction<4

        offset=[d(1:3,1);psai;theta;phi];
else
        offset=[x ;y;z;d(1:3,1)];
end