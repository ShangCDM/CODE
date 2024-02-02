function [rou,fai] = euler2roufai(euler_angle,t0)
%Function: By providing Euler angles and translation vectors,
%calculate the transformation matrix and corresponding Lie algebra

%Input:
%euler_angle=[theita3,theita1,theita2],It means to first rotate around Z to theta3,
%then around new X to theta1, and then around new Y to theta2 in the new coordinate system
%t0 represents the coordinates of the camera origin in the world system.

%OutPut: rou and fai are corresponding Lie algebraic forms

theita3=euler_angle(1);
theita1=euler_angle(2);
theita2=euler_angle(3);
R=[cos(theita2) 0 -sin(theita2);0 1 0;sin(theita2) 0 cos(theita2)]...
  *[1 0 0;0 cos(theita1) sin(theita1);0 -sin(theita1) cos(theita1)]...
  *[cos(theita3) sin(theita3) 0;-sin(theita3) cos(theita3) 0;0 0 1];
R0=inv(R);
T0=[R0 t0;0 0 0 1];
T=inv(T0);
t=T(1:3,4);

theita=acos((trace(R)-1)/2);
a=(1/(2*sin(theita)))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
fai=theita*a;
Jsose=sin(theita)/theita*eye(3,3)+(1-sin(theita)/theita)*a*a'+((1-cos(theita))/theita)*[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
rou=inv(Jsose)*t;
end

