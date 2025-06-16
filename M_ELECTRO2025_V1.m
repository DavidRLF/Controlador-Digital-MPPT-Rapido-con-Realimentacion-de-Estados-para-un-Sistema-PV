clc; clear all; close all;

%% Arreglo PV basado en paneles solares Canadian Solar CS6X-280P
No_Cadenas_Paralelo = 2;
No_Modulos_PorCadena = 18;
Irr_in=0;

%% Parámetros y punto de operación del convertidor Boost
% Arreglo PV (T = 25, Pmp = 10.07 kW, Vmp = 640.8 V)
vpv = 640.8;       % Tensión de entrada mínima
vdc = 1000;        % Tensión de salida deseada (voltaje del bus dc)
Fs = 5e3;          % Frecuencia de conmutación
Ppv = 10.07e3;     % Potencia de entrada
ipv = Ppv/vpv;     % Corriente de entrada
iL = ipv;          % La corriente del inductor se considera igual a ipv
d =(vdc-vpv)/vdc;  % Ciclo de trabajo
Rpv = vpv/ipv;     % Rpv=Rmppt
Idc = ipv*(1-d);   % Calculo de la corriente de carga
Rdc = vdc/(Idc);   % Calculo de la resistencia de carga
Lmin =(d*(1-d)^2*Rdc)/(2*Fs); % Inductancia miníma para mod. cond. cont (MDC)
L = 68.2729*Lmin;  % Para asegurar el modo MDC
ILmax = vpv/((1-d)^2*Rdc)+(vpv*d)/(2*L*Fs); % Corriente máxima del inductor
ILmin = vpv/((1-d)^2*Rdc)-(vpv*d)/(2*L*Fs)  % Corriente mínima del inductor
Cdc = 470e-6;               % Capacitor de salida 
dVdc = (d*vdc)/(Rdc*Cdc*Fs) % Rizo de tensión de salida de 1V
Cpv = 1e-6;                 % Capacitor de entrada   
b = 16;                     % Número de bits del DPWM
Ucmax = (2^b-1);            % Señal de control máxima (0-Ucmax)
Kdpwm = 1/Ucmax;            % Ganancia del DPWM (mod. PWM digital)
UC = d/Kdpwm;               % Definir punto de operación para UC
X1 = iL;                    % Definir punto de operacion para el estado X1 
X2 = vdc;                   % Definir punto de operacion para el estado X2
Ref = ipv;                  % Objetivo a seguir por el controlador

%% Modelo y lazo abierto en discreto del convertidor Boost
% x1 = ipv (corriente del arreglo PV), x2 = vdc (voltaje de salida)
% Matrices de estado
Ts1 = 1/Fs;
Amd = [1 (Ts1*(Kdpwm*UC-1))/L
   -(Ts1*(Kdpwm*UC-1))/Cdc 1-Ts1/(Cdc*Rdc)];
Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];
Cmd = [1/2 0      % Para controlar a x1 = ipv = iL
       0 1];      % Para controlar a x2 = vdc
Dmd = [0
      0];
states = {'x1' 'x2'};
inputs = {'uc'}; 
outputs = {'x1=ipv' 'x2=vdc'};
Sys_Boost_Modelo_Discreto = ss(Amd,Bmd,Cmd,Dmd,Ts1,'statename',states,'inputname',inputs,'outputname',outputs)
tf(Sys_Boost_Modelo_Discreto)
Gz_uc_to_x1=(ans(1,1));
[num_Gz_uc_to_x1, den_Gz_uc_to_x1] = tfdata(Gz_uc_to_x1, 'v');

%% Dis. del cont. del conv. Boost con SF + acción integral en discreto
% Sistema
Cmd_Control = [1 0]; %Control de y = x1 = ipv
Dmd_Control = [0];
Amd = [1 -(Ts1*(1-Kdpwm*UC))/L
   (Ts1*(1-Kdpwm*UC))/Cdc 1-Ts1/(Cdc*Rdc)];
Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];
% Matrices aumentadas Aamd y Bamd
% Aamd para el entorno de simulink usando modelo a bloques 
% ya que la acción integral represanta por un tercer estado se modela 
% explicitamente usando un bloque de suma acumulativa 
% con ganancia Ts de la forma x3[k+1]=x3[k]+Ts(X2ref[k]-x2(k))
Aamd = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
        (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                    0;
        -Ts1                       0                                1 ]; 
% Aamd para el entorno de scrip ya que la acción integral represanta por
% un tercer estado se modela de la forma x3[k+1]=x3[k]+(X2ref[k]-x2(k)) 
% sin incluir a Ts por que ya se contempla en la funcion ss(..)                                             
Aamdprima = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
             (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                  0;
             -1                       0                                1 ]; 
Bamd = [ (Kdpwm*Ts1*X2)/L;
         -(Kdpwm*Ts1*X1)/Cdc;
         0 ];
Eamd = [zeros(size(Aamd,1)-1,1); 1];
Camd = [Cmd_Control 0];
% Polos deseados en z (convertidos desde continuo)
Testd = 0.06;
zeta_d = 1;
p3 = 400;
wn_d = 4 / (zeta_d * Testd);
s1 = -zeta_d * wn_d;
s2 = -zeta_d * wn_d;
s3 = -p3;
z1 = exp(s1 * Ts1);
z2 = exp(s2 * Ts1);
z3 = exp(s3 * Ts1);
Polos_Des_Boost_Mod_Disc = [z1 z2 z3];   %Respetando el método de diseño
% Si usamos el método basado en prueba y error
% z1p = exp(s1p * Ts1); % Si usamos el método basado en prueba y error
% z2p = exp(s2p * Ts1); % Si usamos el método basado en prueba y error
% z3p = exp(s3p * Ts1); % Si usamos el método basado en prueba y error
% Polos_Des_Boost_Mod_Disc = [z1p z2p z3p]; % Si usamos el método basado en prueba y error
% Verificar controlabilidad
Co_Boost_Modelo_Disc = ctrb(Aamd, Bamd);
if rank(Co_Boost_Modelo_Disc) < size(Aamd,1)
    error('El sistema aumentado no es completamente controlable.')
end
% Calcular las ganancias con acker como se usa en el entorno de simulink
Kmd = acker(Aamd, Bamd, Polos_Des_Boost_Mod_Disc);  
K1y2md = Kmd(1:end-1) % Para usar en simulink en lazo cerrado
% Inventir el signo de Ki para evitar accion contraria de la accion integral
% ya que estamos con Vref-Vo, si fuera contrario Vo-Vref no fuese necesario
Kimd = -Kmd(end) % Para usar en simulink en lazo cerrado con modelo a bloques
                 % En caso de simulink usando bloque de funcion se tiene
                 % que multiplicar a Kimd*Ts debido a la forma que se
                 % planteo el modelo en Aamd y la implementacion del
                 % integrador en el bloque de funcion como:
                 % Acc_Int = Ki * ek * Ts + Acc_Int_1;
% Calcular las ganancias con acker como se usa en el entorno del script
Kmdprima = acker(Aamdprima, Bamd, Polos_Des_Boost_Mod_Disc); % Como se usa en script
% Determinar el sistema en lazo cerrado para evaluar en script
Afmd = Aamdprima-Bamd*Kmdprima; 
% Calcular los autovalores (raíces del denominador) del sistema en lazo
% cerrado usando los Kmd del entorno de simulink, los cuales deben ser 
% igual a los polos deseados
Polos_Boost_Lazo_Cerrado_Disc = eig(Aamd-Bamd*Kmd)
% Convertidor Boost con realimentación en espacio de estados en lazo cerrado
% usando los Kmdprima para el entorno del script
SLC_Boost_m_SF_Disc = ss(Afmd,Eamd,Camd,0,Ts1);
% Respuesta del convertidor Boost en lazo abierto asi como en cerrado con SF 
% y acción integral
figure(1)
% Lazo abierto(x1 vs u1)
subplot(2,1,1)
hold on
step((Ucmax-UC) * Gz_uc_to_x1, 'b')     % Discreto
legend('Discreto')
title('Modelo del Convertidor Boost en Lazo Abierto (x1/uc=ipv/(d/Kdpwm))')
ylabel('Corriente ipv [A]')
grid on
% Lazo cerrado (x1 vs u1)
subplot(2,1,2)
hold on
step(Ref * SLC_Boost_m_SF_Disc, 'b')       % Discreto
legend('Discreto')
title('Boost en Lazo Cerrado con SF + Accion Integral')
ylabel('Corriente ipv [A]')
xlabel('Tiempo [s]')
grid on
hold off;

%% Inversor cd-ca  
Fc = 2e3;               % Frecuencia base de la onda triangular portadora (Hz) del inversor
fs = 60;                % Frecuencia base de la red eléctrica (Hz) 
ws = 2*pi*fs;           % Frecuencia base de la red eléctrica (rad/s)
vdc = 1000;             % Voltaje base del bus dc (Volts)
S = 10e3*3/2;           % Potencia aparente base del sistema (VA)
VL = (381.051/2);       % Voltaje de línea base (rms)
Vf = VL/sqrt(3);        % Voltaje de fase base (rms) 
vdc_ref = vdc;          % Voltaje deseado en el bus de dc (Volts)
Lbase = vdc^2/(ws*S);   % Inductancia base del filtro L (H)
%Valor final del inductor (H), Si fc = 2 kHz y THD < 10%, entonces:
L_pu = 0.3;
Linv = L_pu*Lbase;
Linv = 0.0133;           % Valor elegido a prueba y error
Rinv = (Linv*377/fs/2);  % Resistencia interna de Linv
Rinv = 0.0417;
Cinv = 470e-6;           % Capacitor de entrada del inversor
Ts4 = 1/(4*Fc);          % Tiempo de muestro (Segundos)
Td = Ts4/2;              % Tiempo de Retardo de actualización del PWM (Segundos)
Ts_inv = 1/(100*Fc);     % Tiempo de muestreo del PWM y demas bloque que no sean controlador
Vp = 1;                  % Voltaje pico de la portadora del PWM (volts)

%% Diseño de Cont. dig. PI basado en la frecuencia del Inversor cd-ca 
% "Se desprecia el retardo de actualización del PWM"
% Controladores PI del eje dq de corriente
Porcentaje1 = 1;                    %Máximo sobrepaso Mp (%) deseado
Mp1 = Porcentaje1/100;
Zeta1 = sqrt(log(Mp1)^2/(log(Mp1)^2+pi^2)); % Factor de amortiguamiento
tset1 = 10e-3;                      % Tiempo de establecimiento (Seg.) des.                                  
Porcentaje = 2;                     % Error permitido maximo al tset1 
E1 = Porcentaje/100;                 
wn1 = -log(E1)/(Zeta1*tset1);       % Frecuencia natural del sistema (rad/s)
Kp2 = 2*Linv*Vp*Zeta1*wn1-Rinv*Vp;  % Ganancia proporcional
Ki2 = Linv*Vp*wn1^2;                % Ganancia integral
% Controlador PI para lazo de voltaje, en caso de coloar filtro pasab. para id
Tlpf = 0.02;                        % Tiempo de respuesta del filtro pasabajas
Alpha = 2;                          % Maximo Mp de 5%, entonces segun Método de Naslin  
Kp3 = (Cinv)/(Alpha*Tlpf);          % Ganancia proporcional  
Ki3 = (Cinv)/(Alpha^3*Tlpf^2);      % Ganancia integral
% Controlador PI para lazo de voltaje, sin filtro pasab. para id
Porcentaje = 1;                     % Máximo sobrepaso Mp (%) deseado 
Mp2 = Porcentaje/100;               % Factor de amortiguamiento
Zeta2 = sqrt(log(Mp2)^2/(log(Mp2)^2+pi^2));
n = 2;                      % Núm. de veces mas lento que el control dq de corriente   
tset2 = tset1*n;            % Tiempo de establecimiento (Segundos) 
Porcentaje = 4;             % Error permitido máximo a tset2
E2 = Porcentaje/100;
wn2 = -log(E2)/(Zeta2*tset2); % Frecuencia natural del sistema (rad/s)
Kp4 = 2*Cinv*Zeta2*wn2;       % Ganancia proporcional
Ki4 = Cinv*wn2^2;             % Ganancia integral
% Controlador PI para el PLL
Zeta3 = 0.707;                % Amortiguamiento
wn3 = (120*pi)/2;             % Frecuencia de la red 
Kp5 = (2*Zeta3*wn3);          % Ganancia proporcional
Ki5 = wn3^2;                  % Ganancia integral
% Calculo de las ganancias de los controladores digitales
KI2 = Ki2*Ts4;
KP2 = Kp2-KI2/2;
KI3 = Ki3*Ts4;
KP3 = Kp3-KI3/2;
KI4 = Ki4*Ts4;
KP4 = Kp4-KI4/2;
KI5 = Ki5*Ts_inv;
KP5 = Kp5*Ts_inv-KI5/2;

%% Trasformador de 50 KVA (como si fuera un edificio pequeño)
Ptrafo   = 100e3;   % Potencia nominal [W]
Ftrafo   = fs;      % Frecuencia de operación [Hz] 
VLLsec   = 33000;   % Voltaje de línea-línea del lado de la red [Vrms]
VLLprim  = VL;      % Voltaje de línea-línea del lado del inversor [Vrms]

%% Red eléctrica
VLngrid = 33000/sqrt(3);    % Voltaje de linea a neutro [rms]
Fred    = fs;               % Frecuencia de la red [Hz]
In = 1e6/(1.73*(Vf));
Icc_calc = In/(5/100);

% Constantes del control SF usadas para el JCR
K2 = 6.75342153674108;
K1 = [1084.96778587508,-41.3560928961939];

