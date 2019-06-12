function xP = filtro_kalman(v)

I = eye(3);

u = v(1);
y = [v(2) ; v(3)];

P_old = [v(4) v(5) v(6);
         v(7) v(8) v(9);
         v(10) v(11) v(12)];
     
x_old = [v(13); v(14); v(15)];     

% Sistema Linearizado Discretizado 
Ad = eye(3);

Bd = [1; 1; 1];

C = [1 1 1; 1 1 1];

% Parametros do Filtro de Kalman (P0 = 100000000*[1; 0; 0; 0; 1; 0; 0; 0; 1])
V1 = diag([0.0001 1 1]);
r = 100;               
V2 = r*diag([1 1]);

% Measurement update
L = (P_old * C') / (C*P_old*C'+V2);
x = x_old + L * (y - C*x_old);
P = (I-L*C)*P_old;

% Time update
x = Ad * x + Bd * u;
P = Ad * P * Ad' + V1;
xP = [x(1); x(2); x(3); P(1,1); P(1,2); P(1,3); P(2,1); P(2,2); P(2,3); P(3,1); P(3,2); P(3,3)];

end
