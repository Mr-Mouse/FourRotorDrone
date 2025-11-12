clc;
clear;
drone_params.d = 0.2223;         % (m) 轴距
drone_params.m = 1.0230;         % (kg) 质量
drone_params.cT = 1.4865e-07;    % (N/rmp^2) 推力系数
drone_params.cM = 2.9250e-09;    % (Nm/rmp^2) 力矩系数
drone_params.Jxx = 0.0095;       % (kgm^2) 绕X轴的转动惯量
drone_params.Jyy = 0.0095;       % (kgm^2) 绕Y轴的转动惯量  
drone_params.Jzz = 0.0186;       % (kgm^2) 绕Z轴的转动惯量
drone_params.Jrp = 3.7882e-06;   % (kgm^2) 螺旋桨转动惯量
drone_params.CR = 80.5840;       % (rpm) 电机动态响应参数
drone_params.wb = 976.2;         % (rpm) 
drone_params.wmax = 6500;        % (rpm) 
drone_params.Tm = 0.076;         % (s) 电机时间常数
drone_params.g = 9.8;            % (s) 重力加速度