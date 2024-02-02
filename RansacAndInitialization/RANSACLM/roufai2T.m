function T = roufai2T(rou,fai)
%Function£ºTransforming the Lie algebraic form into a transformation matrix

theita=norm(fai);
a=fai/theita;
R=cos(theita)*eye(3,3)+(1-cos(theita))*(a)*(a')+...
      sin(theita)*[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
Jsose=sin(theita)/theita*eye(3,3)+(1-sin(theita)/theita)*(a)*(a')+...
    ((1-cos(theita))/theita)*[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
t=Jsose*rou;
T=[R t;0 0 0 1];
end

