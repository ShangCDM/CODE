function ParaLInitBest = f_RGIA(alpha,beita,gama,u1,v1,u2,v2,u3,v3,Para_min,Para_max,NumParticle,n1)
%Function:Residual-Guided Initialization 

%Input£º
%alpha,beita,gama,u1,v1,u2,v2,u3,v3 are data
%alpha,beita,gama are the joint angles of the boom,arm, and bucket, respectively
%(u1,v1),(u2,v2),(u3,v3)represent the FPPC of the boom, stick, and bucket, respectively
%Para_min,Para_max,NumParticle,n1 are parameters of algorithm

%Output:The optimal initial parameters for the system

%% Geometric parameters of excavator linkage
l1=3710.347;
l2=1615.225;

%% 
Para=zeros(24,NumParticle);
AverageErrorAll=zeros(NumParticle,1);
NumData=length(alpha);
NumForCalcu=n1;
%% Iterative process
saveParaE24=[];
for i=1:NumParticle
    Para(:,i)=rand(1,24).*(Para_max-Para_min)+Para_min;%Random Particle
    fx=Para(1,i);
    u0=Para(2,i);
    fy=Para(3,i);
    v0=Para(4,i);
    k1=Para(11,i);
    k2=Para(12,i);
    p1=Para(13,i);
    p2=Para(14,i);
    k3=Para(15,i);
    lA=Para(16,i);
    lB=Para(17,i);
    lC=Para(18,i);
    alpha1=Para(19,i);
    beita1=Para(20,i);
    gama1=Para(21,i);
    Z1=Para(22,i);
    Z2=Para(23,i);
    Z3=Para(24,i);
    
    %Transformation matrix T
    [rou,fai] = euler2roufai([Para(5,i);Para(6,i);Para(7,i)],[Para(8,i);Para(9,i);Para(10,i)]);
    T=roufai2T(rou,fai);
    PositionForCalcu= randperm(NumData, NumForCalcu);
    
    ECalcu=zeros(6*length(PositionForCalcu),1);%error
    for j=1:length(PositionForCalcu)
        
        %system Model
        
        X1=lA*cos(alpha(PositionForCalcu(j))+alpha1);
        Y1=lA*sin(alpha(PositionForCalcu(j))+alpha1);
        X2=l1*cos(alpha(PositionForCalcu(j)))+lB*cos(alpha(PositionForCalcu(j))+beita(PositionForCalcu(j))+beita1);
        Y2=l1*sin(alpha(PositionForCalcu(j)))+lB*sin(alpha(PositionForCalcu(j))+beita(PositionForCalcu(j))+beita1);
        X3=l1*cos(alpha(PositionForCalcu(j)))+l2*cos(alpha(PositionForCalcu(j))+beita(PositionForCalcu(j)))+lC*cos(alpha(PositionForCalcu(j))+beita(PositionForCalcu(j))+gama(PositionForCalcu(j))+gama1);
        Y3=l1*sin(alpha(PositionForCalcu(j)))+l2*sin(alpha(PositionForCalcu(j))+beita(PositionForCalcu(j)))+lC*sin(alpha(PositionForCalcu(j))+beita(PositionForCalcu(j))+gama(PositionForCalcu(j))+gama1);

        XcYcZc_db=T*[X1;Y1;Z1;1];
        XcYcZc_dg=T*[X2;Y2;Z2;1];
        XcYcZc_cd=T*[X3;Y3;Z3;1];

        xibar1=XcYcZc_db(1)/XcYcZc_db(3);
        yibar1=XcYcZc_db(2)/XcYcZc_db(3);
        xibar2=XcYcZc_dg(1)/XcYcZc_dg(3);
        yibar2=XcYcZc_dg(2)/XcYcZc_dg(3);
        xibar3=XcYcZc_cd(1)/XcYcZc_cd(3);
        yibar3=XcYcZc_cd(2)/XcYcZc_cd(3);
        r1=sqrt(xibar1^2+yibar1^2);
        r2=sqrt(xibar2^2+yibar2^2);
        r3=sqrt(xibar3^2+yibar3^2);
        xdbar1=xibar1*(1+k1*r1^2+k2*r1^4+k3*r1^6)+2*p1*xibar1*yibar1+p2*(r1^2+2*xibar1^2);
        ydbar1=yibar1*(1+k1*r1^2+k2*r1^4+k3*r1^6)+2*p2*xibar1*yibar1+p1*(r1^2+2*yibar1^2);
        xdbar2=xibar2*(1+k1*r2^2+k2*r2^4+k3*r2^6)+2*p1*xibar2*yibar2+p2*(r2^2+2*xibar2^2);
        ydbar2=yibar2*(1+k1*r2^2+k2*r2^4+k3*r2^6)+2*p2*xibar2*yibar2+p1*(r2^2+2*yibar2^2);    
        xdbar3=xibar3*(1+k1*r3^2+k2*r3^4+k3*r3^6)+2*p1*xibar3*yibar3+p2*(r3^2+2*xibar3^2);
        ydbar3=yibar3*(1+k1*r3^2+k2*r3^4+k3*r3^6)+2*p2*xibar3*yibar3+p1*(r3^2+2*yibar3^2);    

        uv1=[fx 0 u0;0 fy v0;0 0 1]*[xdbar1;ydbar1;1];
        uv2=[fx 0 u0;0 fy v0;0 0 1]*[xdbar2;ydbar2;1];
        uv3=[fx 0 u0;0 fy v0;0 0 1]*[xdbar3;ydbar3;1];

        u_db=uv1(1);
        v_db=uv1(2);
        u_dg=uv2(1);
        v_dg=uv2(2);
        u_cd=uv3(1);
        v_cd=uv3(2); 
        
        ECalcu(6*(j-1)+1,1)=u_db-u1(PositionForCalcu(j));
        ECalcu(6*(j-1)+2,1)=v_db-v1(PositionForCalcu(j));
        ECalcu(6*(j-1)+3,1)=u_dg-u2(PositionForCalcu(j));
        ECalcu(6*(j-1)+4,1)=v_dg-v2(PositionForCalcu(j));
        ECalcu(6*(j-1)+5,1)=u_cd-u3(PositionForCalcu(j));
        ECalcu(6*(j-1)+6,1)=v_cd-v3(PositionForCalcu(j));         
    end
    
    AverageError=sqrt(sum(ECalcu.^2)/length(ECalcu));%average error
    AverageErrorAll(i)=AverageError;
   
    
    %save
    saveParaE24=[saveParaE24 Para(:,i)];

end

[minValue, minIndex] = min(AverageErrorAll);%Particle corresponding to minimum error

ParaEInitBest=saveParaE24(:,minIndex);
ParaLInitBest = f_ParaE2ParaL(ParaEInitBest);%OutPut

end

