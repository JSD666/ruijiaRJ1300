function newpoint=exchange(waypoint,type,robot_ID)
%type 1：需要转换的点
%2：不需要转换，相对于机器人本身运动的点
syms w;
w=[waypoint(1,1:3),1]';
v=[waypoint(1,4:6)]';
a=100;
b=450;
    switch robot_ID
        case 'robot_a'%主手，有
            switch type
            %%相对于激光生成路点的坐标变换
                case 1
                %R=rot('z',pi);
                T=trans('x',+a)*trans('y',+b);
                %先平移后旋转
                newxyz=T*w;

            %%固定点的坐标转换
                case 2
                newxyz=w;
            end

        case 'robot_b'
            switch type
            %%相对于激光生成路点的坐标变换
                case 1
                %R=rot('z',pi);
                T=trans('x',-a)*trans('y',-b);
                newxyz=T*w;

            %%固定点的坐标转换
                case 2
                newxyz=w;
            end      
    end
newaxis_angle=v+[0;0;pi];
newxyz;

newpoint=[newxyz(1:3) ,newaxis_angle];