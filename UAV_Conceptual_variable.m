%Developed by Alan Ramirez, GDP Large UAV Humanitarian Aid. 
%October 2017 University of Southampton
%Last updated 25/10/2017


%Design variables
m_aircraft = 6.8; %[kg]  Total mass of aircraft 
S = 0.78; %[m^2] Wing surface 
AR = 7; %Aspect Ratio
e = 1.6; %Oswald efficiency factor
Cd0 = 0.01; % []Zero lift Drag Coefficient
m_bat = 0.97; %[kg] Mass of battery
rho = 1.225; % [Kg/m^3] Altitude density 
n_prop = 0.25; %propulsive efficiency

plotStyle = {'-','--',':', '-.', '-'}; %
%Preliminary calculations
%------------------------------------------------
for Cd0=0.01:0.002:0.02;
E_bat = 6.22*m_bat^2+172.16*m_bat-9.57; %[Wh] Battery capacity 
W_aircraft = m_aircraft*9.80665; % Weight Kg
U = 5:0.2:30; %[m/s] cruising velocity 
k = (pi*e*AR)^-1;
%k = 0.11; %insert here for a specific value of k
Cl = 2*W_aircraft/(S*rho)*(U.^-2); %Lift Coefficient
Cd = (Cd0+k*Cl.^2); %Drag Coefficient
q = 0.5*rho*U.^2*S; %Dynamic Pressure

%Velocities
U_mD = ((2*W_aircraft)/(rho*S))^0.5*(k/Cd0)^0.25; %Speed required for max Range
U_mP = ((2*W_aircraft)/(rho*S))^0.5*(k/(3*Cd0))^0.25;

%Drag 
D_zero_lift = q.*(Cd0); %Drag Cd0 
D_lift = q.*4*k*(W_aircraft/(rho*S))^2.*U.^-4; %Induced Lift Drag
D_total = D_zero_lift + D_lift;  %Total drag
D_mD = 2*W_aircraft*(Cd0*k)^0.5; %= min(D_total);5
D_mP = 4*W_aircraft*(k*Cd0/3)^0.5; %= min(D_total

%Plot Thrust Required
plot(U, D_total,plotStyle{i}); 
title('Thrust required');
xlabel('Velocity (m/s)');
ylabel('Drag (N)');
grid on; 
%hold on;
plot(U_mP, D_mP, 's', 'MarkerEdgeColor','k','MarkerFaceColor', 'w');
%hold on;
plot(U_mD, D_mD,'k*');
legendInfo{i} = ['Cd0 = ' num2str(i)]; % or whatever is appropriate
hold on

end

legend(legendInfo);
%legend(legendInfo, 'Min Power Drag (Thrust max Endurance)','Min Drag (Thrust max Range)');
% %Power
% P_zero_lift = U.*D_zero_lift;
% P_lift = U.*D_lift;
% P_total = U.*D_total;
% P_mD = 2*W_aircraft*((2*W_aircraft)/(rho*S))^0.5*(Cd0*k^3)^0.25; %= min(D_total)*U_mD;
% P_mP = W_aircraft*((2*W_aircraft)/(rho*S))^0.5*(4*Cd0)/(3*Cd0/k)^0.75; %= min(P_total)
% 
% %Plot power required
% %figure;
% plot(U, P_total,'k');
% title('Power Required');
% xlabel('Velocity (m/s)');
% ylabel('Power (W)');
% grid on;
% hold on;
% plot(U_mP, P_mP, 's', 'MarkerEdgeColor','k','MarkerFaceColor', 'w'); %Minimum point
% hold on;
% plot(U_mD, P_mD, 'k*');
% %legend('Total Power(W)','Power for max Endurance','Power for max range' );
% 
% 
% 
% %Range
% m_bat_var = 0.1:0.1:2.04;
% m_payload = 2.04 - m_bat_var; %2.04 is 30% of the mass of aircraft,the variable
% E_bat_var = 6.22*m_bat_var.^2+172.16*m_bat_var-9.57;
% R = n_prop*E_bat_var*3600/(2*W_aircraft)*((rho*S)/(2*W_aircraft))^0.5*U_mD/(Cd0*k^3)^0.25;
% 
% R_max = n_prop*E_bat*3600/(2*W_aircraft)*(1/(k*Cd0))^0.5 % Same as R_max = n_prop*E_bat*3600/(2*W_aircraft)*((rho*S)/(2*W_aircraft))^0.5*U_mD/(Cd0*k^3)^0.25;
% R_U_var = n_prop*E_bat*3600*U./(P_total);
% 
% 
% %Plot Range vs Payload
% figure;
% semilogy(m_payload, R/1000, 'LineWidth',1);
% title('Range vs Payload');
% xlabel('Payload Mass [kg]');
% ylabel('Range [Km]');
% grid on;
% hold on;
% 
% %Display results
% disp('');
% disp('Performance Results');
% disp('----------------------------------');
% disp(['Power for max Endurance: ', num2str(P_mP), ' [W]']);
% disp(['Thrust for max Endurance: ', num2str(D_mP), ' [N]']);
% disp(['Velocity for max Endurance: ', num2str(U_mP), ' [m/s]']);
% disp(['Power for max Range: ', num2str(P_mD), ' [W]']);
% disp(['Thrust for max Range: ', num2str(D_mD), ' [N]']);
% disp(['Velocity for max Range: ', num2str(U_mD), ' [m/s]']);
% disp(['Max Range: ', num2str(R_max/1000), ' [Km]']);

