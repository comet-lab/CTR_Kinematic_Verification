function R = quaternions2R(Q)

q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);

R = [ 2*q1^2-1+2*q2^2  ,  2*(q2*q3+q1*q4)  ,  2*(q2*q4-q1*q3);
      2*(q2*q3-q1*q4)  ,  2*q1^2-1+2*q3^2  ,  2*(q3*q4+q1*q2);
      2*(q2*q4+q1*q3)  ,  2*(q3*q4-q1*q2)  ,  2*q1^2-1+2*q4^2];