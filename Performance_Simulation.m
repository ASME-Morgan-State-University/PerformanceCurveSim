function P = vehicleParameters() 
%============================================================== 
% vehicleParameters.m 
% 
% Stores all vehicle, battery, rider, motor and simulation 
% parameters for the eHPVC Optimization Model. 
% 
% Author: 
%============================================================== 
%% ----------------------------- 
% Vehicle 
%------------------------------ 
P.bikeMass = 34;              % kg 
P.riderMass = 82;             % kg 
P.totalMass = P.bikeMass + P.riderMass; 
%% ----------------------------- 
% Aerodynamics 
%------------------------------ 
P.CdA = 0.30;                 % m^2 
P.rho = 1.225;                % kg/m^3 
P.Crr = 0.005; 
%% ----------------------------- 
% Gravity 
%------------------------------ 
P.g = 9.81; 
%% ----------------------------- 
% Wheel 
%------------------------------ 
P.wheelDiameter = 26;         % inches 
P.wheelRadius = 0.3302;       % meters 
%% ----------------------------- 
% Battery 
%------------------------------ 
P.batteryVoltage = 48;        % Volts 
P.batteryCapacityWh = 480;    % Watt-hours 
P.batteryEnergy = P.batteryCapacityWh * 3600; 
P.lowVoltageCutoff = 41;      % Volts 
%% ----------------------------- 
% Motor (Bafang G020) 
%------------------------------ 
P.motorRatedPower = 500;      % W 
P.motorPeakPower = 500;       % Controller limited 
P.motorEfficiency = 0.90; 

P.motorMaxTorque = 45;        % Nm 
P.controllerCurrentLimit = P.motorPeakPower / P.batteryVoltage; 
%% ----------------------------- 
% Rider 
%------------------------------ 
P.averageRiderPower = 200;    % W 
P.maxSprintPower = 325;       % W 
P.recoveryPower = 140;        % W 
%% ----------------------------- 
% Simulation 
%------------------------------ 
P.dt = 0.1; 
P.simulationTime = 900;       % seconds 
%% ----------------------------- 
% Speed Controller 
%------------------------------ 
P.Kp = 350; 
%% ----------------------------- 
% Race Target Speed 
%------------------------------ 
P.cruiseSpeedMPH = 25; 
%% ----------------------------- 
% Optimization Ranges 
%------------------------------ 
P.study.riderPower = 100:25:300; 
P.study.CdA = 0.20:0.025:0.40; 
P.study.mass = 25:2:40; 
P.study.Crr = 0.003:0.001:0.007; 
P.study.cruiseSpeed = 18:2:30; 
P.study.batteryCapacity = [360 480 600 720]; 
%% ----------------------------- 
% Course Parameters 
%------------------------------ 
P.turn1Start = 60; 
P.turn1End = 90; 
P.turn2Start = 250; 

P.turn2End = 280; 
P.turnSpeed1 = 10; 
P.turnSpeed2 = 14; 
end 
function course = createCourse(P) 
%============================================================== 
% createCourse.m 
% 
% Creates the target speed profile for the race. 
% 
% Output: 
%   course.time 
%   course.targetSpeed (m/s) 
% 
%============================================================== 
dt = P.dt; 
time = 0:dt:P.simulationTime; 
targetSpeed = zeros(size(time)); 
for i = 1:length(time) 
   t = time(i); 
   %---------------------------------------------------------- 
   % Default straightaway 
   %---------------------------------------------------------- 
   speedMPH = P.cruiseSpeedMPH + 2*sin(2*pi*0.003*t); 
   %---------------------------------------------------------- 
   % Turn 1 
   %---------------------------------------------------------- 
   if t >= P.turn1Start && t < P.turn1End 
       speedMPH = P.turnSpeed1; 
   end 
   %---------------------------------------------------------- 
   % Accelerate after Turn 1 
   %---------------------------------------------------------- 
   if t >= P.turn1End && t < P.turn1End+40 
       speedMPH = P.turnSpeed1 + ((t-P.turn1End)/40) * (P.cruiseSpeedMPH-P.turnSpeed1); 
   end 
   %---------------------------------------------------------- 
   % Turn 2 
   %---------------------------------------------------------- 
   if t >= P.turn2Start && t < P.turn2End 

       speedMPH = P.turnSpeed2; 
   end 
   %---------------------------------------------------------- 
   % Accelerate after Turn 2 
   %---------------------------------------------------------- 
   if t >= P.turn2End && t < P.turn2End+30 
       speedMPH = P.turnSpeed2 + ((t-P.turn2End)/30) * (P.cruiseSpeedMPH-P.turnSpeed2); 
   end 
   %---------------------------------------------------------- 
   % Convert mph → m/s 
   %---------------------------------------------------------- 
   targetSpeed(i) = speedMPH * 0.44704; 
end 
course.time = time; 
course.targetSpeed = targetSpeed; 
end 
function riderPower = riderModel(t, speed, targetSpeed, P) 
%============================================================== 
% riderModel.m 
% 
% Dynamic rider power model for eHPVC simulation. 
% 
% Inputs: 
%   t            - Current simulation time (s) 
%   speed        - Current vehicle speed (m/s) 
%   targetSpeed  - Target course speed (m/s) 
%   P            - Parameter structure 
% 
% Output: 
%   riderPower   - Rider mechanical power (W) 
% 
%============================================================== 
%% Base cruising effort 
riderPower = P.averageRiderPower; 
%% ----------------------------------------------------------- 
% Sprint if below target speed 
%% ----------------------------------------------------------- 
speedError = targetSpeed - speed; 
if speedError > 1.0 
   riderPower = P.maxSprintPower; 
end 

%% ----------------------------------------------------------- 
% Recovery after hard effort 
%% ----------------------------------------------------------- 
persistent sprintTimer 
if isempty(sprintTimer) 
   sprintTimer = 0; 
end 
if riderPower == P.maxSprintPower 
   sprintTimer = 10; 
end 
if sprintTimer > 0 
   sprintTimer = sprintTimer - P.dt; 
end 
if sprintTimer <= 0 && speedError < 0.3 
   riderPower = P.recoveryPower; 
end 
%% ----------------------------------------------------------- 
% Long-term fatigue model 
%% ----------------------------------------------------------- 
fatigue = 1 - 0.10*(t/P.simulationTime); 
fatigue = max(fatigue,0.85); 
riderPower = riderPower * fatigue; 
%% ----------------------------------------------------------- 
% Never exceed limits 
%% ----------------------------------------------------------- 
riderPower = min(riderPower,P.maxSprintPower); 
riderPower = max(riderPower,50); 
end 
function motor = motorModel(throttle,speed,P) 
%============================================================== 
% motorModel.m 
% 
% Realistic Bafang G020 Hub Motor Model 
% 
% Inputs 
%   throttle    0-1 
%   speed       m/s 
%   P           parameter structure 
% 
% Outputs 
%   motor.power         Mechanical Output Power (W) 
%   motor.batteryPower  Battery Input Power (W) 
%   motor.current       Battery Current (A) 
%   motor.force         Driving Force (N) 

%   motor.torque        Hub Torque (Nm) 
% 
%============================================================== 
%% ----------------------------- 
% Limit throttle 
%% ----------------------------- 
throttle = max(throttle,0); 
throttle = min(throttle,1); 
%% ----------------------------- 
% Requested motor power 
%% ----------------------------- 
requestedPower = throttle * P.motorPeakPower; 
%% ----------------------------- 
% Motor power limit 
%% ----------------------------- 
motorPower = min(requestedPower,P.motorPeakPower); 
motorPower = max(motorPower,0); 
%% ----------------------------- 
% Battery power 
%% ----------------------------- 
batteryPower = motorPower / P.motorEfficiency; 
%% ----------------------------- 
% Battery current 
%% ----------------------------- 
batteryCurrent = batteryPower / P.batteryVoltage; 
%% ----------------------------- 
% Current limiting 
%% ----------------------------- 
batteryCurrent = min(batteryCurrent, P.controllerCurrentLimit); 
batteryPower = batteryCurrent * P.batteryVoltage; 
motorPower = batteryPower * P.motorEfficiency; 
%% ----------------------------- 
% Wheel force 
%% ----------------------------- 
vehicleSpeed = max(speed,1); 
driveForce =  motorPower / vehicleSpeed; 
%% ----------------------------- 

% Torque 
%% ----------------------------- 
motorTorque = driveForce * P.wheelRadius; 
%% ----------------------------- 
% Torque limiting 
%% ----------------------------- 
motorTorque = min(motorTorque,... 
       P.motorMaxTorque); 
driveForce = motorTorque / P.wheelRadius; 
%% ----------------------------- 
% Recalculate power 
%% ----------------------------- 
motorPower =  driveForce * vehicleSpeed; 
batteryPower = motorPower / P.motorEfficiency; 
batteryCurrent = batteryPower / P.batteryVoltage; 
%% ----------------------------- 
% Store Results 
%% ----------------------------- 
motor.power = motorPower; 
motor.batteryPower = batteryPower; 
motor.current = batteryCurrent; 
motor.force = driveForce; 
motor.torque = motorTorque; 
end 
function battery = batteryModel(battery,motor,P) 
%============================================================== 
% batteryModel.m 
% 
% Updates battery state for one simulation time step. 
% 
% Inputs 
%   battery   Battery structure from previous time step 
%   motor     Motor structure from motorModel() 
%   P         Parameter structure 
% 
% Output 
%   battery   Updated battery structure 
% 

%============================================================== 
%% Battery power draw 
battery.power = motor.batteryPower; 
%% Battery current 
battery.current = motor.current; 
%% Remove energy from battery 
energyUsed = battery.power * P.dt; 
battery.energy = battery.energy - energyUsed; 
%% Prevent negative energy 
battery.energy = max(battery.energy,0); 
%% State of Charge (SOC) 
battery.SOC = 100 * battery.energy / (P.batteryCapacityWh*3600); 
%% --------------------------------------------------------- 
% Simple Open Circuit Voltage Model 
% 
% 100% SOC -> 54.6 V 
%   0% SOC -> 41.0 V 
%% --------------------------------------------------------- 
battery.voltage = P.lowVoltageCutoff + (54.6-P.lowVoltageCutoff) *  battery.SOC/100; 
%% --------------------------------------------------------- 
% Internal Resistance Model 
%% --------------------------------------------------------- 
internalResistance = 0.15;      % Ohms 
voltageDrop = battery.current * internalResistance; 
battery.loadedVoltage = battery.voltage - voltageDrop; 
%% Prevent voltage below cutoff 
battery.loadedVoltage = max(battery.loadedVoltage, P.lowVoltageCutoff); 
%% --------------------------------------------------------- 
% Estimated Remaining Runtime 
%% --------------------------------------------------------- 
if battery.power > 1 
   battery.runtimeHours = (battery.energy/3600) / battery.power; 
else 

   battery.runtimeHours = inf; 
end 
%% --------------------------------------------------------- 
% Energy Used (Wh) 
%% --------------------------------------------------------- 
battery.energyUsedWh = P.batteryCapacityWh - battery.energy/3600; 
end 
function results = simulateRace(P) 
%============================================================== 
% simulateRace.m 
% 
% Main race simulation 
% 
% Uses: 
%   createCourse.m 
%   riderModel.m 
%   motorModel.m 
%   batteryModel.m 
% 
% Returns: 
%   results structure 
% 
%============================================================== 
%% Create course 
course = createCourse(P); 
time = course.time; 
targetSpeed = course.targetSpeed; 
N = length(time); 
%% Initialize battery 
battery = initializeBattery(P); 
%% Allocate Arrays 
speed           = zeros(1,N); 
acceleration    = zeros(1,N); 
distance        = zeros(1,N); 
throttle        = zeros(1,N); 
motorPower      = zeros(1,N); 
motorTorque     = zeros(1,N); 
motorForce      = zeros(1,N); 
batteryPower    = zeros(1,N); 
batteryCurrent  = zeros(1,N); 
SOC             = zeros(1,N); 
batteryVoltage  = zeros(1,N); 

riderPower      = zeros(1,N); 
dragForce       = zeros(1,N); 
rollingForce    = zeros(1,N); 
%% Initial Conditions 
speed(1) = 5;          % m/s (~11 mph) 
SOC(1) = 100; 
batteryVoltage(1) = 54.6; 
%% ============================================================ 
% Main Simulation Loop 
%% ============================================================ 
for i = 2:N 
   %% ----------------------------------- 
   % Rider 
   %% ----------------------------------- 
   riderPower(i) = riderModel(time(i),speed(i-1),targetSpeed(i), P); 
   %% ----------------------------------- 
   % Speed Controller 
   %% ----------------------------------- 
   speedError = targetSpeed(i)-speed(i-1); 
   throttle(i) = P.Kp*speedError/P.motorPeakPower; 
   throttle(i)=max(0,min(1,throttle(i))); 
   %% ----------------------------------- 
   % Motor 
   %% ----------------------------------- 
   motor = motorModel(throttle(i),speed(i-1),P); 
   %% Store Motor Values 
   motorPower(i) = motor.power; 
   motorTorque(i)= motor.torque; 
   motorForce(i)= motor.force; 
   %% ----------------------------------- 
   % Forces 
   %% ----------------------------------- 
   dragForce(i)=0.5*P.rho*P.CdA*speed(i-1)^2; 
   rollingForce(i)=P.Crr*P.totalMass*P.g; 
   riderForce = riderPower(i)/max(speed(i-1),1); 
   %% Net Force 
   netForce = motorForce(i) + riderForce - dragForce(i) - rollingForce(i); 
   %% ----------------------------------- 
   % Vehicle Dynamics 
   %% ----------------------------------- 
   acceleration(i)= netForce/P.totalMass; 
   speed(i)= speed(i-1) + acceleration(i)*P.dt; 
   speed(i)=max(speed(i),0); 
   %% ----------------------------------- 
   % Distance 
   %% ----------------------------------- 
   distance(i)= distance(i-1)+ speed(i)*P.dt; 
   %% ----------------------------------- 
   % Battery 
   %% ----------------------------------- 
   battery = batteryModel(battery,motor, P); 
   batteryPower(i)= battery.power; 
   batteryCurrent(i)= battery.current; 
   SOC(i)= battery.SOC; 
   batteryVoltage(i)= battery.loadedVoltage; 
   %% Stop if battery empty 
   if battery.energy <= 0 
       fprintf("Battery depleted.\n"); 
       speed(i:end)=0; 
       break 

   end 
end 
%% ============================================================ 
% Summary Statistics 
%% ============================================================ 
results.averageSpeedMPH = mean(speed)*2.23694; 
results.maxSpeedMPH = max(speed)*2.23694; 
results.distanceMiles = distance(end)/1609; 
results.energyUsedWh = battery.energyUsedWh; 
results.remainingSOC = battery.SOC; 
results.remainingEnergyWh = battery.energy/3600; 
results.time = time; 
%% ============================================================ 
% Store Arrays 
%% ============================================================ 
results.speed = speed; 
results.targetSpeed = targetSpeed; 
results.acceleration = acceleration; 
results.distance = distance; 
results.motorPower = motorPower; 
results.motorTorque = motorTorque; 
results.motorForce = motorForce; 
results.riderPower = riderPower; 
results.batteryPower = batteryPower; 
results.batteryCurrent = batteryCurrent; 
results.batteryVoltage = batteryVoltage; 
results.SOC = SOC; 
results.dragForce = dragForce; 
results.rollingForce = rollingForce; 
results.throttle = throttle; 
end 
function studies = parameterStudy() 
%============================================================== 
% Parameter Study 
% 
% Runs multiple simulations while varying one parameter 
%============================================================== 
close all 

%% Base Parameters 
P = vehicleParameters(); 
%% ========================================================== 
% Rider Power Study 
%% ========================================================== 
fprintf("\nRunning Rider Power Study...\n") 
rider = P.study.riderPower; 
for i = 1:length(rider) 
   Ptemp = P; 
   Ptemp.averageRiderPower = rider(i); 
   R = simulateRace(Ptemp); 
   studies.rider.avgSpeed(i) = R.averageSpeedMPH; 
   studies.rider.energy(i) = R.energyUsedWh; 
   studies.rider.SOC(i) = R.remainingSOC; 
   studies.rider.distance(i) = R.distanceMiles; 
end 
%% ========================================================== 
% CdA Study 
%% ========================================================== 
fprintf("Running Aerodynamic Study...\n") 
CdA = P.study.CdA; 
for i = 1:length(CdA) 
   Ptemp = P; 
   Ptemp.CdA = CdA(i); 
   R = simulateRace(Ptemp); 
   studies.CdA.avgSpeed(i) = R.averageSpeedMPH; 
   studies.CdA.energy(i) = R.energyUsedWh; 
   studies.CdA.SOC(i) = R.remainingSOC; 
   studies.CdA.distance(i) = R.distanceMiles; 
end 
%% ========================================================== 
% Mass Study 
%% ========================================================== 
fprintf("Running Mass Study...\n") 
mass = P.study.mass; 
for i = 1:length(mass) 
   Ptemp = P; 
   Ptemp.bikeMass = mass(i); 
   Ptemp.totalMass = Ptemp.bikeMass + Ptemp.riderMass; 
   R = simulateRace(Ptemp); 
   studies.mass.avgSpeed(i) = R.averageSpeedMPH; 
   studies.mass.energy(i) = R.energyUsedWh; 
   studies.mass.SOC(i) = R.remainingSOC; 

   studies.mass.distance(i) = R.distanceMiles; 
end 
%% ========================================================== 
% Rolling Resistance Study 
%% ========================================================== 
fprintf("Running Tire Study...\n") 
Crr = P.study.Crr; 
for i = 1:length(Crr) 
   Ptemp = P; 
   Ptemp.Crr = Crr(i); 
   R = simulateRace(Ptemp); 
   studies.Crr.avgSpeed(i) = R.averageSpeedMPH; 
   studies.Crr.energy(i) = R.energyUsedWh; 
   studies.Crr.SOC(i) = R.remainingSOC; 
   studies.Crr.distance(i) = R.distanceMiles; 
end 
%% ========================================================== 
% Speed Strategy Study 
%% ========================================================== 
fprintf("Running Speed Strategy Study...\n") 
speed = P.study.cruiseSpeed; 
for i = 1:length(speed) 
   Ptemp = P; 
   Ptemp.cruiseSpeedMPH = speed(i); 
   R = simulateRace(Ptemp); 
   studies.speed.avgSpeed(i) = R.averageSpeedMPH; 
   studies.speed.energy(i) = R.energyUsedWh; 
   studies.speed.SOC(i) = R.remainingSOC; 
   studies.speed.distance(i) = R.distanceMiles; 
end 
%% ========================================================== 
% Battery Study 
%% ========================================================== 
fprintf("Running Battery Study...\n") 
battery = P.study.batteryCapacity; 
for i = 1:length(battery) 
   Ptemp = P; 
   Ptemp.batteryCapacityWh = battery(i); 
   Ptemp.batteryEnergy = battery(i)*3600; 
   R = simulateRace(Ptemp); 
   studies.battery.avgSpeed(i) = R.averageSpeedMPH; 
   studies.battery.energy(i) = R.energyUsedWh; 
   studies.battery.SOC(i) = R.remainingSOC; 

   studies.battery.distance(i) = R.distanceMiles; 
end 
%% Save X-Axis Values 
studies.rider.x = rider; 
studies.CdA.x = CdA; 
studies.mass.x = mass; 
studies.Crr.x = Crr; 
studies.speed.x = speed; 
studies.battery.x = battery; 
fprintf("\nAll parameter studies complete.\n") 
end 
function plotting(results, studies) 
%============================================================== 
% plotting.m 
% 
% Generates all simulation and parameter study figures. 
% 
% Inputs: 
%   results - Output from simulateRace() 
%   studies - Output from parameterStudy() 
% 
%============================================================== 
%% ---------------------------- 
% Speed Profile 
%----------------------------- 
figure('Name','Speed Profile'); 
plot(results.time/60,results.speed*2.23694,'LineWidth',2) 
hold on 
plot(results.time/60,results.targetSpeed*2.23694,'--','LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('Speed (mph)') 
title('Vehicle Speed vs Target Speed') 
legend('Vehicle','Target') 
%% ---------------------------- 
% Acceleration 
%----------------------------- 
figure('Name','Acceleration') 
plot(results.time/60,results.acceleration,'LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('Acceleration (m/s^2)') 
title('Acceleration') 
%% ---------------------------- 
% Motor Power 
%----------------------------- 
figure('Name','Motor Power') 
plot(results.time/60,results.motorPower,'LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('Power (W)') 
title('Motor Output Power') 
%% ---------------------------- 
% Rider Power 
%----------------------------- 
figure('Name','Rider Power') 
plot(results.time/60,results.riderPower,'LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('Power (W)') 
title('Rider Power') 
%% ---------------------------- 
% Battery Current 
%----------------------------- 
figure('Name','Battery Current') 
plot(results.time/60,results.batteryCurrent,'LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('Current (A)') 
title('Battery Current') 
%% ---------------------------- 
% Battery Voltage 
%----------------------------- 
figure('Name','Battery Voltage') 
plot(results.time/60,results.batteryVoltage,'LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('Voltage (V)') 
title('Battery Voltage') 
%% ---------------------------- 
% Battery SOC 
%----------------------------- 
figure('Name','Battery SOC') 
plot(results.time/60,results.SOC,'LineWidth',2) 
grid on 
xlabel('Time (minutes)') 
ylabel('SOC (%)') 
title('Battery State of Charge') 
%% ---------------------------- 
% Distance vs SOC 
%----------------------------- 
figure('Name','SOC vs Distance') 
plot(results.distance/1609,results.SOC,'LineWidth',2) 
grid on 
xlabel('Distance (miles)') 
ylabel('SOC (%)') 
title('Battery State of Charge vs Distance') 
%% ===================================================== 
% PARAMETER STUDIES 
%% ===================================================== 
%% Rider Power 
figure('Name','Rider Study') 
subplot(2,2,1) 
plot(studies.rider.x,studies.rider.avgSpeed,'-o','LineWidth',2) 
grid on 
xlabel('Average Rider Power (W)') 
ylabel('Average Speed (mph)') 
title('Speed vs Rider Power') 
subplot(2,2,2) 
plot(studies.rider.x,studies.rider.energy,'-o','LineWidth',2) 
grid on 
xlabel('Average Rider Power (W)') 
ylabel('Energy Used (Wh)') 
title('Energy vs Rider Power') 
subplot(2,2,3) 
plot(studies.rider.x,studies.rider.SOC,'-o','LineWidth',2) 
grid on 
xlabel('Average Rider Power (W)') 
ylabel('Remaining SOC (%)') 
title('SOC vs Rider Power') 
subplot(2,2,4) 
plot(studies.rider.x,studies.rider.distance,'-o','LineWidth',2) 
grid on 
xlabel('Average Rider Power (W)') 
ylabel('Distance (miles)') 
title('Distance vs Rider Power') 
%% Aerodynamics 
figure('Name','CdA Study') 
plot(studies.CdA.x,studies.CdA.avgSpeed,'-o','LineWidth',2) 
grid on 
xlabel('CdA') 
ylabel('Average Speed (mph)') 
title('Average Speed vs Aerodynamic Drag') 
%% Vehicle Mass 
figure('Name','Mass Study') 
plot(studies.mass.x,studies.mass.avgSpeed,'-o','LineWidth',2) 
grid on 
xlabel('Bike Mass (kg)') 
ylabel('Average Speed (mph)') 
title('Average Speed vs Bike Mass') 
%% Rolling Resistance 
figure('Name','Rolling Resistance') 
plot(studies.Crr.x,studies.Crr.avgSpeed,'-o','LineWidth',2) 
grid on 
xlabel('Rolling Resistance') 
ylabel('Average Speed (mph)') 
title('Average Speed vs Rolling Resistance') 
%% Battery Capacity 
figure('Name','Battery Study') 
plot(studies.battery.x,studies.battery.distance,'-o','LineWidth',2) 
grid on 
xlabel('Battery Capacity (Wh)') 
ylabel('Distance (miles)') 
title('Distance vs Battery Capacity') 
%% Speed Strategy 
figure('Name','Speed Strategy') 
subplot(2,1,1) 
plot(studies.speed.x,studies.speed.energy,'-o','LineWidth',2) 
grid on 
xlabel('Cruise Speed (mph)') 
ylabel('Energy Used (Wh)') 
title('Energy Consumption vs Cruise Speed') 
subplot(2,1,2) 
plot(studies.speed.x,studies.speed.distance,'-o','LineWidth',2) 
grid on 
xlabel('Cruise Speed (mph)') 
ylabel('Distance (miles)') 
title('Distance vs Cruise Speed') 
end 
function main()
%==============================================================
% main.m
%
% Main entry point for the eHPVC Performance Optimization Model.
%
% Runs:
%   1. Base race simulation
%   2. Parameter studies
%   3. Performance plots
%   4. Sensitivity analysis
%   5. Optimization report
%   6. CSV export
%
% To run the complete model, use:
%   main
%
%==============================================================

clc;
close all;

fprintf("=========================================");
fprintf(" eHPVC Performance Optimization");
fprintf("=========================================");

%% Load Vehicle Parameters
P = vehicleParameters();

%% Run Base Simulation
fprintf("Running Base Simulation...");
results = simulateRace(P);

%% Display Base Results
fprintf("");
fprintf("-------------------------------------");
fprintf("Simulation Results");
fprintf("-------------------------------------");
fprintf("Average Speed       : %.2f mph", results.averageSpeedMPH);
fprintf("Maximum Speed       : %.2f mph", results.maxSpeedMPH);
fprintf("Distance Travelled  : %.2f miles", results.distanceMiles);
fprintf("Energy Used         : %.2f Wh", results.energyUsedWh);
fprintf("Remaining Battery   : %.2f %%", results.remainingSOC);
fprintf("-------------------------------------");

%% Run Parameter Studies
fprintf("Running Optimization Studies...");
studies = parameterStudy();

%% Generate Figures
fprintf("Generating Figures...");
plotting(results, studies);

%% Sensitivity Analysis
fprintf("Running Sensitivity Analysis...");
sensitivityAnalysis(studies);

%% Optimization Report
fprintf("Generating Optimization Report...");
optimizationReport(studies);

%% Export Results
fprintf("Exporting Results...");
exportResults(results, studies);

%% Multi-Lap Race Simulation
fprintf("\nRunning Multi-Lap Race Simulation...\n");

numLaps = 3;
race = lapSimulator(P, numLaps);

%% Completion Message
fprintf("");
fprintf("=========================================");
fprintf(" Simulation Complete");
fprintf("=========================================");

end 
function sensitivityAnalysis(studies) 
%============================================================== 
% sensitivityAnalysis.m 
% 
% Determines which design parameters have the greatest influence 
% on vehicle performance. 
% 
%============================================================== 
%% Rider Power 
riderChange = max(studies.rider.avgSpeed) - min(studies.rider.avgSpeed); 
%% CdA 
CdAChange = max(studies.CdA.avgSpeed) - min(studies.CdA.avgSpeed); 
%% Mass 
massChange = max(studies.mass.avgSpeed) - min(studies.mass.avgSpeed); 
%% Rolling Resistance 
CrrChange = max(studies.Crr.avgSpeed) - min(studies.Crr.avgSpeed); 
%% Battery Capacity 
batteryChange = max(studies.battery.distance) - min(studies.battery.distance); 
%% Speed Strategy 
speedChange = max(studies.speed.energy) - min(studies.speed.energy); 
%% Create Table 
names = { 
'Rider Power' 
'Aerodynamic Drag' 
'Bike Mass' 
'Rolling Resistance' 
'Battery Capacity' 
'Cruise Speed' 
}; 
effects = [ 

riderChange 
CdAChange 
massChange 
CrrChange 
batteryChange 
speedChange 
]; 
T = table(names,effects); 
disp(' ') 
disp('==============================') 
disp('Sensitivity Analysis') 
disp('==============================') 
disp(T) 
%% Horizontal Bar Chart 
figure('Name','Sensitivity Analysis') 
barh(effects) 
grid on 
yticks(1:length(names)) 
yticklabels(names) 
xlabel('Effect Magnitude') 
title('Sensitivity of Vehicle Performance') 
end 
function optimizationReport(studies) 
%============================================================== 
% optimizationReport.m 
% 
% Summarizes all parameter studies 
% 
%============================================================== 
clc 
fprintf('\n') 
fprintf('===============================================\n') 
fprintf('        eHPVC PERFORMANCE SUMMARY\n') 
fprintf('===============================================\n\n') 
%% Rider Power 
[bestSpeed,idx] = max(studies.rider.avgSpeed); 
fprintf('RIDER POWER STUDY\n') 
fprintf('---------------------------------\n') 
fprintf('Best Average Speed : %.2f mph\n',bestSpeed) 
fprintf('Best Rider Power   : %.0f W\n',studies.rider.x(idx)) 
fprintf('Battery Remaining  : %.2f %%\n\n',studies.rider.SOC(idx)) 
%% CdA 
[bestSpeed,idx] = max(studies.CdA.avgSpeed); 
fprintf('AERODYNAMIC STUDY\n') 

fprintf('---------------------------------\n') 
fprintf('Best Average Speed : %.2f mph\n',bestSpeed) 
fprintf('Best CdA           : %.3f\n',studies.CdA.x(idx)) 
fprintf('Battery Remaining  : %.2f %%\n\n',studies.CdA.SOC(idx)) 
%% Mass 
[bestSpeed,idx] = max(studies.mass.avgSpeed); 
fprintf('MASS STUDY\n') 
fprintf('---------------------------------\n') 
fprintf('Best Average Speed : %.2f mph\n',bestSpeed) 
fprintf('Best Bike Mass     : %.1f kg\n',studies.mass.x(idx)) 
fprintf('Battery Remaining  : %.2f %%\n\n',studies.mass.SOC(idx)) 
%% Rolling Resistance 
[bestSpeed,idx] = max(studies.Crr.avgSpeed); 
fprintf('ROLLING RESISTANCE STUDY\n') 
fprintf('---------------------------------\n') 
fprintf('Best Average Speed : %.2f mph\n',bestSpeed) 
fprintf('Best Crr           : %.4f\n',studies.Crr.x(idx)) 
fprintf('Battery Remaining  : %.2f %%\n\n',studies.Crr.SOC(idx)) 
%% Speed Strategy 
[bestSpeed,idx] = max(studies.speed.avgSpeed); 
fprintf('SPEED STRATEGY STUDY\n') 
fprintf('---------------------------------\n') 
fprintf('Best Average Speed : %.2f mph\n',bestSpeed) 
fprintf('Cruise Speed       : %.1f mph\n',studies.speed.x(idx)) 
fprintf('Battery Remaining  : %.2f %%\n\n',studies.speed.SOC(idx)) 
%% Battery Study 
[maxSOC,idx] = max(studies.battery.SOC); 
fprintf('BATTERY STUDY\n') 
fprintf('---------------------------------\n') 
fprintf('Best Capacity      : %.0f Wh\n',studies.battery.x(idx)) 
fprintf('Remaining Battery  : %.2f %%\n',maxSOC) 
fprintf('Distance           : %.2f miles\n\n',studies.battery.distance(idx)) 
fprintf('===============================================\n') 
fprintf('End of Optimization Report\n') 
fprintf('===============================================\n') 
end 
function exportResults(results, studies) 
%============================================================== 
% exportResults.m 
% 
% Exports simulation results to CSV files. 
% 
%============================================================== 
fprintf('Exporting Results...\n'); 

%% ----------------------------- 
% Main Simulation Data 
%% ----------------------------- 
simulationTable = table(results.time(:),results.speed(:),results.targetSpeed(:), results.acceleration(:),results.distance(:),results.motorPower(:),results.motorTorque(:),results.riderPower(:),results.batteryPower(:),results.batteryCurrent(:),results.batteryVoltage(:),results.SOC(:),'VariableNames',{'Time_s','Speed_mps','TargetSpeed_mps','Acceleration','Distance_m','MotorPower_W','MotorTorque_Nm','RiderPower_W','BatteryPower_W','BatteryCurrent_A','BatteryVoltage_V','SOC_percent'}); 
writetable(simulationTable,'SimulationResults.csv'); 
%% ----------------------------- 
% Rider Study 
%% ----------------------------- 
T = table(studies.rider.x(:),studies.rider.avgSpeed(:),studies.rider.energy(:),studies.rider.SOC(:),studies.rider.distance(:),'VariableNames',{'RiderPower','AverageSpeed','EnergyUsed','SOC','Distance'}); 
writetable(T,'RiderStudy.csv'); 
%% ----------------------------- 
% CdA Study 
%% ----------------------------- 
T = table(studies.CdA.x(:),studies.CdA.avgSpeed(:),studies.CdA.energy(:),studies.CdA.SOC(:),studies.CdA.distance(:),'VariableNames',{'CdA','AverageSpeed','EnergyUsed','SOC','Distance'}); 
writetable(T,'CdAStudy.csv'); 
%% ----------------------------- 
% Mass Study 
%% ----------------------------- 
T = table(studies.mass.x(:),studies.mass.avgSpeed(:),studies.mass.energy(:),studies.mass.SOC(:),studies.mass.distance(:),'VariableNames',{'Mass','AverageSpeed','EnergyUsed','SOC','Distance'}); 
writetable(T,'MassStudy.csv'); 
%% ----------------------------- 
% Rolling Resistance Study 
%% ----------------------------- 
T = table(studies.Crr.x(:),studies.Crr.avgSpeed(:),studies.Crr.energy(:),... 
   studies.Crr.SOC(:),studies.Crr.distance(:),'VariableNames',{'RollingResistance','AverageSpeed','EnergyUsed','SOC','Distance'}); 
writetable(T,'RollingResistanceStudy.csv'); 
%% ----------------------------- 
% Speed Study 
%% ----------------------------- 
T = table(studies.speed.x(:),studies.speed.avgSpeed(:),studies.speed.energy(:),studies.speed.SOC(:),studies.speed.distance(:),'VariableNames',{'CruiseSpeed','AverageSpeed','EnergyUsed','SOC','Distance'}); 
writetable(T,'SpeedStudy.csv'); 
%% ----------------------------- 
% Battery Study 
%% ----------------------------- 
T = table(studies.battery.x(:),studies.battery.avgSpeed(:),studies.battery.energy(:),studies.battery.SOC(:),studies.battery.distance(:),'VariableNames',{'BatteryCapacity','AverageSpeed','EnergyUsed','SOC','Distance'}); 
writetable(T,'BatteryStudy.csv'); 
fprintf('Export Complete.\n'); 
end 
function race = lapSimulator(P,numLaps) 
%============================================================== 
% lapSimulator.m 
% 
% Simulates a complete multi-lap race. 
% 
% Inputs: 
%   P        Vehicle parameter structure 
%   numLaps  Number of laps 
% 
% Output: 
%   race     Race summary structure 
% 
%============================================================== 
fprintf('\n') 
fprintf('Beginning Multi-Lap Simulation...\n') 
%% Initialize 
totalDistance = 0; 
totalTime = 0; 
totalEnergy = 0; 
lapTime = zeros(1,numLaps); 
lapDistance = zeros(1,numLaps); 
lapEnergy = zeros(1,numLaps); 
lapSOC = zeros(1,numLaps); 
%% Run Each Lap 
for lap = 1:numLaps 
   fprintf('Running Lap %d of %d...\n',lap,numLaps) 
   results = simulateRace(P); 
   lapTime(lap) = results.time(end); 
   lapDistance(lap) = results.distanceMiles; 
   lapEnergy(lap) = results.energyUsedWh; 
   lapSOC(lap) = results.remainingSOC; 
   totalDistance = totalDistance + results.distanceMiles; 
   totalTime = totalTime + results.time(end); 
   totalEnergy = totalEnergy + results.energyUsedWh; 
   %% Stop if battery depleted 
   if results.remainingSOC <= 0 
       fprintf('Battery depleted during Lap %d\n',lap) 
       break 
   end 
end 
%% Summary 
race.totalDistance = totalDistance; 
race.totalTime = totalTime; 
race.totalEnergy = totalEnergy; 
race.averageLapTime = mean(lapTime(lapTime>0)); 
race.averageSpeed = totalDistance/(totalTime/3600); 
race.finalSOC = lapSOC(find(lapSOC>0,1,'last')); 
race.lapTime = lapTime; 
race.lapDistance = lapDistance; 
race.lapEnergy = lapEnergy; 
race.lapSOC = lapSOC; 
%% Print Results 
fprintf('\n') 
fprintf('========================================\n') 
fprintf('Race Summary\n') 
fprintf('========================================\n') 
fprintf('Total Distance : %.2f miles\n',race.totalDistance) 
fprintf('Total Time     : %.2f minutes\n',race.totalTime/60) 
fprintf('Average Speed  : %.2f mph\n',race.averageSpeed) 
fprintf('Energy Used    : %.2f Wh\n',race.totalEnergy) 
fprintf('Remaining SOC  : %.2f %%\n',race.finalSOC) 
fprintf('========================================\n') 
%% Plots 
figure 
plot(1:numLaps,lapTime/60,'o-','LineWidth',2) 
grid on 
xlabel('Lap') 
ylabel('Lap Time (minutes)') 
title('Lap Time During Race') 
figure 
plot(1:numLaps,lapSOC,'o-','LineWidth',2) 
grid on 
xlabel('Lap') 
ylabel('Battery SOC (%)') 
title('Battery Remaining After Each Lap') 
figure 
plot(1:numLaps,lapEnergy,'o-','LineWidth',2) 
grid on 
xlabel('Lap') 
ylabel('Energy Used (Wh)') 
title('Energy Consumption Per Lap') 
end 
function battery = initializeBattery(P) 
% Initialize battery state using parameters in P 
battery.energy = P.batteryCapacityWh * 3600; % Joules 
battery.power = 0; 
battery.current = 0; 
battery.SOC = 100; 
battery.voltage = 54.6; 
battery.loadedVoltage = battery.voltage; 
battery.runtimeHours = inf; 
battery.energyUsedWh = 0; 
end
%% ============================================================
% eHPVC FIGURES
% =============================================================
%% 1. Vehicle Speed vs Target Speed
figure('Name','Vehicle Speed vs Target Speed');
plot(results.time/60, results.speed*2.23694, 'LineWidth',2);
hold on;
plot(results.time/60, results.targetSpeed*2.23694, '--','LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Speed (mph)');
title('Vehicle Speed vs Target Speed');
legend('Actual Vehicle Speed','Target Speed','Location','best');
%% 2. Acceleration vs Time
figure('Name','Acceleration');
plot(results.time/60, results.acceleration,'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Acceleration (m/s^2)');
title('Vehicle Acceleration vs Time');
%% 3. Motor Power vs Time
figure('Name','Motor Power');
plot(results.time/60, results.motorPower, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Motor Power (W)');
title('Motor Power vs Time');
%% 4. Rider Power vs Time
figure('Name','Rider Power');
plot(results.time/60, results.riderPower,'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Rider Power (W)');
title('Rider Power vs Time');
%% 5. Battery Current vs Time
figure('Name','Battery Current');
plot(results.time/60, results.batteryCurrent,'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Battery Current (A)');
title('Battery Current vs Time');
%% 6. Battery Voltage vs Time
figure('Name','Battery Voltage');
plot(results.time/60, results.batteryVoltage,'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Battery Voltage (V)');
title('Battery Voltage vs Time');
%% 7. Battery SOC vs Time
figure('Name','Battery State of Charge');
plot(results.time/60, results.SOC,'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Battery SOC (%)');
title('Battery State of Charge vs Time');
%% 8. Battery SOC vs Distance
figure('Name','SOC vs Distance');
plot(results.distance/1609.344, results.SOC,'LineWidth',2);
grid on;
xlabel('Distance (miles)');
ylabel('Battery SOC (%)');
title('Battery State of Charge vs Distance');
%% ============================================================
% PARAMETER STUDY FIGURES
% ============================================================
%% 9. Rider Power Study
figure('Name','Rider Power Study');
subplot(2,2,1);
plot(studies.rider.x, studies.rider.avgSpeed,'-o','LineWidth',2);
grid on;
xlabel('Average Rider Power (W)');
ylabel('Average Speed (mph)');
title('Average Speed vs Rider Power');
subplot(2,2,2);
plot(studies.rider.x, studies.rider.energy,'-o','LineWidth',2);
grid on;
xlabel('Average Rider Power (W)');
ylabel('Energy Used (Wh)');
title('Energy Used vs Rider Power');
subplot(2,2,3);
plot(studies.rider.x, studies.rider.SOC,'-o','LineWidth',2);
grid on;
xlabel('Average Rider Power (W)');
ylabel('Remaining SOC (%)');
title('Remaining SOC vs Rider Power');
subplot(2,2,4);
plot(studies.rider.x, studies.rider.distance,'-o','LineWidth',2);
grid on;
xlabel('Average Rider Power (W)');
ylabel('Distance (miles)');
title('Distance vs Rider Power');
%% 10. Aerodynamic Drag Study
figure('Name','Aerodynamic Drag Study');
plot(studies.CdA.x, studies.CdA.avgSpeed,'-o','LineWidth',2);
grid on;
xlabel('CdA (m^2)');
ylabel('Average Speed (mph)');
title('Average Speed vs Aerodynamic Drag');
%% 11. Vehicle Mass Study
figure('Name','Vehicle Mass Study');
plot(studies.mass.x, studies.mass.avgSpeed,'-o','LineWidth',2);
grid on;
xlabel('Bike Mass (kg)');
ylabel('Average Speed (mph)');
title('Average Speed vs Bike Mass');
%% 12. Rolling Resistance Study
figure('Name','Rolling Resistance Study');
plot(studies.Crr.x, studies.Crr.avgSpeed,'-o','LineWidth',2);
grid on;
xlabel('Rolling Resistance Coefficient');
ylabel('Average Speed (mph)');
title('Average Speed vs Rolling Resistance');
%% 13. Battery Capacity Study
figure('Name','Battery Capacity Study');
plot(studies.battery.x, studies.battery.distance,'-o','LineWidth',2);
grid on;
xlabel('Battery Capacity (Wh)');
ylabel('Distance (miles)');
title('Distance vs Battery Capacity');
%% 14. Cruise Speed Strategy Study
figure('Name','Cruise Speed Strategy');
subplot(2,1,1);
plot(studies.speed.x, studies.speed.energy,'-o','LineWidth',2);
grid on;
xlabel('Cruise Speed (mph)');
ylabel('Energy Used (Wh)');
title('Energy Consumption vs Cruise Speed');
subplot(2,1,2);
plot(studies.speed.x, studies.speed.distance,'-o','LineWidth',2);
grid on;
xlabel('Cruise Speed (mph)');
ylabel('Distance (miles)');
title('Distance vs Cruise Speed');
%% 15. Sensitivity Analysis
figure('Name','Sensitivity Analysis');
riderChange = max(studies.rider.avgSpeed) - min(studies.rider.avgSpeed);
CdAChange = max(studies.CdA.avgSpeed) - min(studies.CdA.avgSpeed);
massChange = max(studies.mass.avgSpeed) - min(studies.mass.avgSpeed);
CrrChange = max(studies.Crr.avgSpeed) - min(studies.Crr.avgSpeed);
batteryChange = max(studies.battery.distance) - min(studies.battery.distance);
speedChange = max(studies.speed.energy) - min(studies.speed.energy);
names = {
    'Rider Power'
    'Aerodynamic Drag'
    'Bike Mass'
    'Rolling Resistance'
    'Battery Capacity'
    'Cruise Speed'
};
effects = [
    riderChange
    CdAChange
    massChange
    CrrChange
    batteryChange
    speedChange
];
barh(effects);
grid on;
yticks(1:length(names));
yticklabels(names);
xlabel('Effect Magnitude');
ylabel('Parameter');
title('Sensitivity of Vehicle Performance');
