function ParaL = f_ParaE2ParaL(ParaE)
%Function: Converting parameter vectors in Euler angle form to parameter vectors in Lie algebra form

N=length(ParaE(1,:));
ParaL=ParaE;

for i=1:N
    [ParaL(5:7,i),ParaL(8:10,i)]=euler2roufai(ParaE(5:7,i),ParaE(8:10,i));
end

end