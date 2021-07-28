function cost= minimizeE2(E2,lambda,work1,HpposHp,Ib,Ibm,Ict)
% nm=0;% THIS IS INPUT OVERRIDE VALUE
% display(E2')
size(E2);
size(HpposHp);
work2=(eye(size(HpposHp))-HpposHp)*E2;
E=work1+work2;
dthr= -lambda*E;
hr=norm(Ict*dthr);
cost = hr;
end

