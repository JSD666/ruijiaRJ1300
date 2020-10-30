function newRPY =rad_RPY_to_arm(p1,p2,LIDARTOMA_ID)
%p1,p2为测量的两个点
%LIDARTOMA_ID为对应函数名LIDARTOMA_123

 %%
%XYZ工具坐标系，xyz基座坐标系
%建立工具坐标系

 X=p2-p1;
negativ_y=[0 -1 0];
negativ_z=[0 0 -1];

switch LIDARTOMA_ID
    case 'LIDARTOMA1'
        %工具坐标系X轴与基座坐标系-Y轴叉乘，获得的归一化向量作为工具坐标系的Z轴，再将工具坐标系的Z轴与工具坐标系的X轴叉乘，得到工具坐标系的Y轴。
         Z=cross_product(X,negativ_y);
         Y=cross_product(Z,X);

    case 'LIDARTOMA2'
        %工具坐标系X轴与基座坐标系-Z轴叉乘，获得的归一化向量作为工具坐标系的Y轴，再将工具坐标系的X轴与工具坐标系的Y轴叉乘，得到工具坐标系的Z轴。
         Y=cross_product(X,negativ_z);
         Z=cross_product(X,Y);
         
    case 'LIDARTOMA3'
        %工具坐标系X轴与基座坐标系-Z轴叉乘，获得的归一化向量作为工具坐标系的Y轴，再将工具坐标系的X轴与工具坐标系的Y轴叉乘，得到工具坐标系的Z轴。
         Y=cross_product(X,negativ_z);
         Z=cross_product(X,Y);
end

%%
%求工具坐标系单位向量和旋转矩阵
x=X/sqrt(X(1)^2 + X(2)^2 + X(3)^2);
y=Y/sqrt(Y(1)^2 + Y(2)^2 + Y(3)^2);
z=Z/sqrt(Z(1)^2 + Z(2)^2 + Z(3)^2);

RR=[x',y',z'];
%RR=[x;y;z];
%%
%求欧拉角
if (RR(3,1)~=1 && RR(3,1)~=-1)
    sprintf('不等于1')
    theta=-asin(RR(3,1));
    psai=atan2(RR(3,2)/cos(theta),RR(3,3)/cos(theta));
    phi=atan2(RR(2,1)/cos(theta),RR(1,1)/cos(theta));
else
    phi=0;
    if(RR(3,1)==-1)
        theta=pi/2;
        sprintf('等于-1')
        psai=phi+atan2(RR(1,2),RR(1,3));
    else
        sprintf('等于1')
        theta=-pi/2;
        psai=-phi+atan2(-RR(1,2),-RR(1,3));
    end 
    
end

% psai=atan2(RR(2,3),RR(3,3));
% c2=sqrt(RR(1,1)^2+RR(1,2)^2);
% theta=atan2(-RR(1,3),c2);
% s1=sin(psai);
% c1=cos(psai);
% phi=atan2(s1*RR(3,1)-c1*RR(2,1),c1*RR(2,2)-s1*RR(3,2));

%%
%result

RR
newRPY=[psai theta phi];
end