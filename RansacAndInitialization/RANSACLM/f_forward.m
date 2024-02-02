function EAve = f_forward(ParaL,alpha,beita,gama,u1,v1,u2,v2,u3,v3)

%Function:Residual-Guided Initialization 

%Input£º
%alpha,beita,gama,u1,v1,u2,v2,u3,v3 are data
%alpha,beita,gama are the joint angles of the boom,arm, and bucket, respectively
%(u1,v1),(u2,v2),(u3,v3)represent the FPPC of the boom, stick, and bucket, respectively
%ParaL is system parameter vector

%Output:Fitting residuals

%% Geometric parameters of excavator linkage
l1=3710.347;
l2=1615.225;

%%
%system parameter
fx_hat=ParaL(1);
u0_hat=ParaL(2);
fy_hat=ParaL(3);
v0_hat=ParaL(4);
rou1_hat=ParaL(5);
rou2_hat=ParaL(6);
rou3_hat=ParaL(7);
fai1_hat=ParaL(8);
fai2_hat=ParaL(9);
fai3_hat=ParaL(10);
k1_hat=ParaL(11);
k2_hat=ParaL(12);
p1_hat=ParaL(13);
p2_hat=ParaL(14);
k3_hat=ParaL(15);
lA_hat=ParaL(16);
lB_hat=ParaL(17);
lC_hat=ParaL(18);
alpha1_hat=ParaL(19);
beita1_hat=ParaL(20);
gama1_hat=ParaL(21);
Z1_hat=ParaL(22);
Z2_hat=ParaL(23);
Z3_hat=ParaL(24);

%%
K_hat=[fx_hat 0 u0_hat;0 fy_hat v0_hat;0 0 1];

T_hat=roufai2T([rou1_hat;rou2_hat;rou3_hat],[fai1_hat;fai2_hat;fai3_hat]);

Nu=length(u1);
E=zeros(6*Nu,1);
EAve=zeros(Nu,1);
for i=1:Nu

    %World coordinates of three feature points
    XYZ_1_hat=[lA_hat*cos(alpha(i)+alpha1_hat);...
               lA_hat*sin(alpha(i)+alpha1_hat);Z1_hat];%World coordinates of the center of mass of the target boom 
    XYZ_2_hat=[l1*cos(alpha(i))+lB_hat*cos(alpha(i)+beita(i)+beita1_hat);...
               l1*sin(alpha(i))+lB_hat*sin(alpha(i)+beita(i)+beita1_hat);...
               Z2_hat];
    XYZ_3_hat=[l1*cos(alpha(i))+l2*cos(alpha(i)+beita(i))+...
               lC_hat*cos(alpha(i)+beita(i)+gama(i)+gama1_hat);...
               l1*sin(alpha(i))+l2*sin(alpha(i)+beita(i))+...
               lC_hat*sin(alpha(i)+beita(i)+gama(i)+gama1_hat);...
               Z3_hat];

    %Camera coordinates
    XcYcZc_1_hat=T_hat*[XYZ_1_hat;1];
    XcYcZc_2_hat=T_hat*[XYZ_2_hat;1];
    XcYcZc_3_hat=T_hat*[XYZ_3_hat;1];

    %Ideal normalized coordinates
    xi_bar_1_hat=XcYcZc_1_hat(1)/XcYcZc_1_hat(3);
    yi_bar_1_hat=XcYcZc_1_hat(2)/XcYcZc_1_hat(3);
    r_1_hat=sqrt(xi_bar_1_hat^2+yi_bar_1_hat^2);
    xi_bar_2_hat=XcYcZc_2_hat(1)/XcYcZc_2_hat(3);
    yi_bar_2_hat=XcYcZc_2_hat(2)/XcYcZc_2_hat(3);
    r_2_hat=sqrt(xi_bar_2_hat^2+yi_bar_2_hat^2);
    xi_bar_3_hat=XcYcZc_3_hat(1)/XcYcZc_3_hat(3);
    yi_bar_3_hat=XcYcZc_3_hat(2)/XcYcZc_3_hat(3);
    r_3_hat=sqrt(xi_bar_3_hat^2+yi_bar_3_hat^2);

    %Distorted normalized coordinates
    xd_bar_1_hat=xi_bar_1_hat*(1+k1_hat*r_1_hat^2+k2_hat*r_1_hat^4+...
                 k3_hat*r_1_hat^6)+2*p1_hat*xi_bar_1_hat*yi_bar_1_hat+...
                 p2_hat*(r_1_hat^2+2*xi_bar_1_hat^2);
    yd_bar_1_hat=yi_bar_1_hat*(1+k1_hat*r_1_hat^2+k2_hat*r_1_hat^4+...
                 k3_hat*r_1_hat^6)+2*p2_hat*xi_bar_1_hat*yi_bar_1_hat+...
                 p1_hat*(r_1_hat^2+2*yi_bar_1_hat^2);
    xd_bar_2_hat=xi_bar_2_hat*(1+k1_hat*r_2_hat^2+k2_hat*r_2_hat^4+...
                 k3_hat*r_2_hat^6)+2*p1_hat*xi_bar_2_hat*yi_bar_2_hat+...
                 p2_hat*(r_2_hat^2+2*xi_bar_2_hat^2);
    yd_bar_2_hat=yi_bar_2_hat*(1+k1_hat*r_2_hat^2+k2_hat*r_2_hat^4+...
                 k3_hat*r_2_hat^6)+2*p2_hat*xi_bar_2_hat*yi_bar_2_hat+...
                 p1_hat*(r_2_hat^2+2*yi_bar_2_hat^2);
    xd_bar_3_hat=xi_bar_3_hat*(1+k1_hat*r_3_hat^2+k2_hat*r_3_hat^4+...
                 k3_hat*r_3_hat^6)+2*p1_hat*xi_bar_3_hat*yi_bar_3_hat+...
                 p2_hat*(r_3_hat^2+2*xi_bar_3_hat^2);
    yd_bar_3_hat=yi_bar_3_hat*(1+k1_hat*r_3_hat^2+k2_hat*r_3_hat^4+...
                 k3_hat*r_3_hat^6)+2*p2_hat*xi_bar_3_hat*yi_bar_3_hat+...
                 p1_hat*(r_3_hat^2+2*yi_bar_3_hat^2);       

    %Pixel coordinates
    UV_1_hat=K_hat*[xd_bar_1_hat;yd_bar_1_hat;1];
    UV_2_hat=K_hat*[xd_bar_2_hat;yd_bar_2_hat;1];
    UV_3_hat=K_hat*[xd_bar_3_hat;yd_bar_3_hat;1];

    %The error of two coordinates of three feature points, totaling 6
    E(6*(i-1)+1,1)=UV_1_hat(1)-u1(i);
    E(6*(i-1)+2,1)=UV_1_hat(2)-v1(i);
    E(6*(i-1)+3,1)=UV_2_hat(1)-u2(i);
    E(6*(i-1)+4,1)=UV_2_hat(2)-v2(i);
    E(6*(i-1)+5,1)=UV_3_hat(1)-u3(i);
    E(6*(i-1)+6,1)=UV_3_hat(2)-v3(i);
    EAve(i)=sqrt(((E(6*(i-1)+1,1))^2+(E(6*(i-1)+2,1))^2+(E(6*(i-1)+3,1))^2+(E(6*(i-1)+4,1))^2+(E(6*(i-1)+5,1))^2+(E(6*(i-1)+6,1))^2)/6);
end
end

