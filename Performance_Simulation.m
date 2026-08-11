%% eHPVC Performance Optimization Model
% Save this entire file as: main.m
% Run by pressing Run in MATLAB or typing: main

clear;
clc;
close all;

fprintf('=========================================\n');
fprintf(' eHPVC Performance Optimization\n');
fprintf('=========================================\n\n');

P = vehicleParameters();

fprintf('Running Base Simulation...\n');
results = simulateRace(P);

fprintf('\n-----------------------------------------\n');
fprintf('Simulation Results\n');
fprintf('-----------------------------------------\n');
fprintf('Average Speed      : %.2f mph\n', results.averageSpeedMPH);
fprintf('Maximum Speed      : %.2f mph\n', results.maxSpeedMPH);
fprintf('Distance Travelled : %.2f miles\n', results.distanceMiles);
fprintf('Energy Used        : %.2f Wh\n', results.energyUsedWh);
fprintf('Remaining Battery  : %.2f %%\n', results.remainingSOC);
fprintf('-----------------------------------------\n\n');

fprintf('Running Parameter Studies...\n');
studies = parameterStudy(P);

fprintf('Generating Figures...\n');
plotting(results, studies);

fprintf('Running Sensitivity Analysis...\n');
sensitivityAnalysis(studies);

fprintf('Generating Optimization Report...\n');
optimizationReport(studies);

fprintf('Exporting Results...\n');
exportResults(results, studies);

fprintf('\n=========================================\n');
fprintf(' Simulation Complete\n');
fprintf('=========================================\n');


%% ============================================================
function P = vehicleParameters()
% Stores vehicle, battery, rider, motor, simulation, and study parameters.

P.bikeMass = 34;
P.riderMass = 82;
P.totalMass = P.bikeMass + P.riderMass;

P.CdA = 0.30;
P.rho = 1.225;
P.Crr = 0.005;
P.g = 9.81;

P.wheelDiameter = 26;
P.wheelRadius = 0.3302;

P.batteryVoltage = 48;
P.batteryCapacityWh = 480;
P.batteryEnergy = P.batteryCapacityWh * 3600;
P.lowVoltageCutoff = 41;

P.motorRatedPower = 500;
P.motorPeakPower = 500;
P.motorEfficiency = 0.90;
P.motorMaxTorque = 45;
P.controllerCurrentLimit = P.motorPeakPower / P.batteryVoltage;

P.averageRiderPower = 200;
P.maxSprintPower = 325;
P.recoveryPower = 140;

P.dt = 0.1;
P.simulationTime = 900;

P.Kp = 350;
P.cruiseSpeedMPH = 25;

P.study.riderPower = 100:25:300;
P.study.CdA = 0.20:0.025:0.40;
P.study.mass = 25:2:40;
P.study.Crr = 0.003:0.001:0.007;
P.study.cruiseSpeed = 18:2:30;
P.study.batteryCapacity = [360 480 600 720];

P.turn1Start = 60;
P.turn1End = 90;
P.turn2Start = 250;
P.turn2End = 280;
P.turnSpeed1 = 10;
P.turnSpeed2 = 14;
end


%% ============================================================
function course = createCourse(P)
% Creates the target speed profile.

time = 0:P.dt:P.simulationTime;
targetSpeed = zeros(size(time));

for i = 1:length(time)
    t = time(i);

    speedMPH = P.cruiseSpeedMPH + 2*sin(2*pi*0.003*t);

    if t >= P.turn1Start && t < P.turn1End
        speedMPH = P.turnSpeed1;
    elseif t >= P.turn1End && t < P.turn1End + 40
        speedMPH = P.turnSpeed1 + ...
            ((t-P.turn1End)/40) * ...
            (P.cruiseSpeedMPH-P.turnSpeed1);
    end

    if t >= P.turn2Start && t < P.turn2End
        speedMPH = P.turnSpeed2;
    elseif t >= P.turn2End && t < P.turn2End + 30
        speedMPH = P.turnSpeed2 + ...
            ((t-P.turn2End)/30) * ...
            (P.cruiseSpeedMPH-P.turnSpeed2);
    end

    targetSpeed(i) = speedMPH * 0.44704;
end

course.time = time;
course.targetSpeed = targetSpeed;
end


%% ============================================================
function riderPower = riderModel(t, speed, targetSpeed, P)
% Dynamic rider power model.

persistent sprintTimer

if isempty(sprintTimer)
    sprintTimer = 0;
end

riderPower = P.averageRiderPower;
speedError = targetSpeed - speed;

if speedError > 1.0
    riderPower = P.maxSprintPower;
    sprintTimer = 10;
end

if sprintTimer > 0
    sprintTimer = max(0, sprintTimer - P.dt);
elseif speedError < 0.3
    riderPower = P.recoveryPower;
end

fatigue = max(1 - 0.10*(t/P.simulationTime), 0.85);
riderPower = riderPower * fatigue;

riderPower = min(riderPower, P.maxSprintPower);
riderPower = max(riderPower, 50);
end


%% ============================================================
function motor = motorModel(throttle, speed, P)
% Motor model.

throttle = max(0, min(1, throttle));

requestedPower = throttle * P.motorPeakPower;
motorPower = max(0, min(requestedPower, P.motorPeakPower));

batteryPower = motorPower / P.motorEfficiency;
batteryCurrent = batteryPower / P.batteryVoltage;

batteryCurrent = min(batteryCurrent, P.controllerCurrentLimit);
batteryPower = batteryCurrent * P.batteryVoltage;
motorPower = batteryPower * P.motorEfficiency;

vehicleSpeed = max(speed, 1);
driveForce = motorPower / vehicleSpeed;

motorTorque = driveForce * P.wheelRadius;
motorTorque = min(motorTorque, P.motorMaxTorque);

driveForce = motorTorque / P.wheelRadius;
motorPower = driveForce * vehicleSpeed;

batteryPower = motorPower / P.motorEfficiency;
batteryCurrent = batteryPower / P.batteryVoltage;

motor.power = motorPower;
motor.batteryPower = batteryPower;
motor.current = batteryCurrent;
motor.force = driveForce;
motor.torque = motorTorque;
end


%% ============================================================
function battery = initializeBattery(P)
% Initializes battery state.

battery.energy = P.batteryCapacityWh * 3600;
battery.power = 0;
battery.current = 0;
battery.SOC = 100;
battery.voltage = 54.6;
battery.loadedVoltage = 54.6;
battery.runtimeHours = inf;
battery.energyUsedWh = 0;
end


%% ============================================================
function battery = batteryModel(battery, motor, P)
% Updates battery for one simulation time step.

battery.power = motor.batteryPower;
battery.current = motor.current;

battery.energy = max(battery.energy - battery.power*P.dt, 0);

battery.SOC = 100 * battery.energy / (P.batteryCapacityWh*3600);

battery.voltage = P.lowVoltageCutoff + ...
    (54.6-P.lowVoltageCutoff) * battery.SOC/100;

internalResistance = 0.15;
battery.loadedVoltage = max( ...
    battery.voltage - battery.current*internalResistance, ...
    P.lowVoltageCutoff);

if battery.power > 1
    battery.runtimeHours = (battery.energy/3600) / battery.power;
else
    battery.runtimeHours = inf;
end

battery.energyUsedWh = P.batteryCapacityWh - battery.energy/3600;
end


%% ============================================================
function results = simulateRace(P)
% Main race simulation.

course = createCourse(P);
time = course.time;
targetSpeed = course.targetSpeed;
N = length(time);

battery = initializeBattery(P);

speed = zeros(1,N);
acceleration = zeros(1,N);
distance = zeros(1,N);
throttle = zeros(1,N);

motorPower = zeros(1,N);
motorTorque = zeros(1,N);
motorForce = zeros(1,N);

batteryPower = zeros(1,N);
batteryCurrent = zeros(1,N);
batteryVoltage = zeros(1,N);
SOC = zeros(1,N);

riderPower = zeros(1,N);
dragForce = zeros(1,N);
rollingForce = zeros(1,N);

speed(1) = 5;
SOC(1) = battery.SOC;
batteryVoltage(1) = battery.loadedVoltage;

for i = 2:N

    riderPower(i) = riderModel( ...
        time(i), speed(i-1), targetSpeed(i), P);

    speedError = targetSpeed(i) - speed(i-1);

    throttle(i) = P.Kp * speedError / P.motorPeakPower;
    throttle(i) = max(0, min(1, throttle(i)));

    motor = motorModel(throttle(i), speed(i-1), P);

    motorPower(i) = motor.power;
    motorTorque(i) = motor.torque;
    motorForce(i) = motor.force;

    dragForce(i) = 0.5 * P.rho * P.CdA * speed(i-1)^2;
    rollingForce(i) = P.Crr * P.totalMass * P.g;

    riderForce = riderPower(i) / max(speed(i-1), 1);

    netForce = motorForce(i) + riderForce ...
        - dragForce(i) - rollingForce(i);

    acceleration(i) = netForce / P.totalMass;

    speed(i) = max(0, ...
        speed(i-1) + acceleration(i)*P.dt);

    distance(i) = distance(i-1) + speed(i)*P.dt;

    battery = batteryModel(battery, motor, P);

    batteryPower(i) = battery.power;
    batteryCurrent(i) = battery.current;
    batteryVoltage(i) = battery.loadedVoltage;
    SOC(i) = battery.SOC;

    if battery.energy <= 0
        fprintf('Battery depleted at %.1f seconds.\n', time(i));
        speed(i+1:end) = 0;
        distance(i+1:end) = distance(i);
        break;
    end
end

results.averageSpeedMPH = mean(speed)*2.23694;
results.maxSpeedMPH = max(speed)*2.23694;
results.distanceMiles = distance(end)/1609.344;
results.energyUsedWh = battery.energyUsedWh;
results.remainingSOC = battery.SOC;
results.remainingEnergyWh = battery.energy/3600;

results.time = time;
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


%% ============================================================
function studies = parameterStudy(P)
% Runs all parameter studies.

fprintf('\nRunning Rider Power Study...\n');

rider = P.study.riderPower;
studies.rider = runStudy(P, rider, 'rider');

fprintf('Running Aerodynamic Study...\n');

CdA = P.study.CdA;
studies.CdA = runStudy(P, CdA, 'CdA');

fprintf('Running Mass Study...\n');

mass = P.study.mass;
studies.mass = runStudy(P, mass, 'mass');

fprintf('Running Rolling Resistance Study...\n');

Crr = P.study.Crr;
studies.Crr = runStudy(P, Crr, 'Crr');

fprintf('Running Speed Strategy Study...\n');

cruise = P.study.cruiseSpeed;
studies.speed = runStudy(P, cruise, 'speed');

fprintf('Running Battery Study...\n');

battery = P.study.batteryCapacity;
studies.battery = runStudy(P, battery, 'battery');

fprintf('All parameter studies complete.\n');
end


%% ============================================================
function study = runStudy(P, values, type)
% Helper function used by parameterStudy.

n = length(values);

study.x = values;
study.avgSpeed = zeros(1,n);
study.energy = zeros(1,n);
study.SOC = zeros(1,n);
study.distance = zeros(1,n);

for i = 1:n
    Ptemp = P;

    switch type
        case 'rider'
            Ptemp.averageRiderPower = values(i);

        case 'CdA'
            Ptemp.CdA = values(i);

        case 'mass'
            Ptemp.bikeMass = values(i);
            Ptemp.totalMass = Ptemp.bikeMass + Ptemp.riderMass;

        case 'Crr'
            Ptemp.Crr = values(i);

        case 'speed'
            Ptemp.cruiseSpeedMPH = values(i);

        case 'battery'
            Ptemp.batteryCapacityWh = values(i);
            Ptemp.batteryEnergy = values(i)*3600;
    end

    R = simulateRace(Ptemp);

    study.avgSpeed(i) = R.averageSpeedMPH;
    study.energy(i) = R.energyUsedWh;
    study.SOC(i) = R.remainingSOC;
    study.distance(i) = R.distanceMiles;
end
end


%% ============================================================
function plotting(results, studies)
% Generates all simulation and parameter-study figures.

figure('Name','Vehicle Speed vs Target Speed');
plot(results.time/60, results.speed*2.23694, 'LineWidth',2);
hold on;
plot(results.time/60, results.targetSpeed*2.23694, '--','LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Speed (mph)');
title('Vehicle Speed vs Target Speed');
legend('Actual Vehicle Speed','Target Speed','Location','best');

figure('Name','Acceleration');
plot(results.time/60, results.acceleration, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Acceleration (m/s^2)');
title('Vehicle Acceleration vs Time');

figure('Name','Motor Power');
plot(results.time/60, results.motorPower, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Motor Power (W)');
title('Motor Power vs Time');

figure('Name','Rider Power');
plot(results.time/60, results.riderPower, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Rider Power (W)');
title('Rider Power vs Time');

figure('Name','Battery Current');
plot(results.time/60, results.batteryCurrent, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Battery Current (A)');
title('Battery Current vs Time');

figure('Name','Battery Voltage');
plot(results.time/60, results.batteryVoltage, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Battery Voltage (V)');
title('Battery Voltage vs Time');

figure('Name','Battery State of Charge');
plot(results.time/60, results.SOC, 'LineWidth',2);
grid on;
xlabel('Time (minutes)');
ylabel('Battery SOC (%)');
title('Battery State of Charge vs Time');

figure('Name','SOC vs Distance');
plot(results.distance/1609.344, results.SOC, 'LineWidth',2);
grid on;
xlabel('Distance (miles)');
ylabel('Battery SOC (%)');
title('Battery State of Charge vs Distance');

figure('Name','Rider Power Study');
subplot(2,2,1);
plot(studies.rider.x, studies.rider.avgSpeed,'-o','LineWidth',2);
grid on; xlabel('Average Rider Power (W)');
ylabel('Average Speed (mph)');
title('Average Speed vs Rider Power');

subplot(2,2,2);
plot(studies.rider.x, studies.rider.energy,'-o','LineWidth',2);
grid on; xlabel('Average Rider Power (W)');
ylabel('Energy Used (Wh)');
title('Energy Used vs Rider Power');

subplot(2,2,3);
plot(studies.rider.x, studies.rider.SOC,'-o','LineWidth',2);
grid on; xlabel('Average Rider Power (W)');
ylabel('Remaining SOC (%)');
title('Remaining SOC vs Rider Power');

subplot(2,2,4);
plot(studies.rider.x, studies.rider.distance,'-o','LineWidth',2);
grid on; xlabel('Average Rider Power (W)');
ylabel('Distance (miles)');
title('Distance vs Rider Power');

figure('Name','Aerodynamic Drag Study');
plot(studies.CdA.x, studies.CdA.avgSpeed,'-o','LineWidth',2);
grid on; xlabel('CdA (m^2)');
ylabel('Average Speed (mph)');
title('Average Speed vs Aerodynamic Drag');

figure('Name','Vehicle Mass Study');
plot(studies.mass.x, studies.mass.avgSpeed,'-o','LineWidth',2);
grid on; xlabel('Bike Mass (kg)');
ylabel('Average Speed (mph)');
title('Average Speed vs Bike Mass');

figure('Name','Rolling Resistance Study');
plot(studies.Crr.x, studies.Crr.avgSpeed,'-o','LineWidth',2);
grid on; xlabel('Rolling Resistance Coefficient');
ylabel('Average Speed (mph)');
title('Average Speed vs Rolling Resistance');

figure('Name','Battery Capacity Study');
plot(studies.battery.x, studies.battery.distance,'-o','LineWidth',2);
grid on; xlabel('Battery Capacity (Wh)');
ylabel('Distance (miles)');
title('Distance vs Battery Capacity');

figure('Name','Cruise Speed Strategy');
subplot(2,1,1);
plot(studies.speed.x, studies.speed.energy,'-o','LineWidth',2);
grid on; xlabel('Cruise Speed (mph)');
ylabel('Energy Used (Wh)');
title('Energy Consumption vs Cruise Speed');

subplot(2,1,2);
plot(studies.speed.x, studies.speed.distance,'-o','LineWidth',2);
grid on; xlabel('Cruise Speed (mph)');
ylabel('Distance (miles)');
title('Distance vs Cruise Speed');

riderChange = max(studies.rider.avgSpeed) - min(studies.rider.avgSpeed);
CdAChange = max(studies.CdA.avgSpeed) - min(studies.CdA.avgSpeed);
massChange = max(studies.mass.avgSpeed) - min(studies.mass.avgSpeed);
CrrChange = max(studies.Crr.avgSpeed) - min(studies.Crr.avgSpeed);
batteryChange = max(studies.battery.distance) - min(studies.battery.distance);
speedChange = max(studies.speed.energy) - min(studies.speed.energy);

names = {'Rider Power'; 'Aerodynamic Drag'; 'Bike Mass'; ...
         'Rolling Resistance'; 'Battery Capacity'; 'Cruise Speed'};

effects = [riderChange; CdAChange; massChange; CrrChange; ...
           batteryChange; speedChange];

figure('Name','Sensitivity Analysis');
barh(effects);
grid on;
yticks(1:length(names));
yticklabels(names);
xlabel('Effect Magnitude');
ylabel('Parameter');
title('Sensitivity of Vehicle Performance');
end


%% ============================================================
function sensitivityAnalysis(studies)
% Prints a sensitivity-analysis table.

riderChange = max(studies.rider.avgSpeed) - min(studies.rider.avgSpeed);
CdAChange = max(studies.CdA.avgSpeed) - min(studies.CdA.avgSpeed);
massChange = max(studies.mass.avgSpeed) - min(studies.mass.avgSpeed);
CrrChange = max(studies.Crr.avgSpeed) - min(studies.Crr.avgSpeed);
batteryChange = max(studies.battery.distance) - min(studies.battery.distance);
speedChange = max(studies.speed.energy) - min(studies.speed.energy);

names = {'Rider Power'; 'Aerodynamic Drag'; 'Bike Mass'; ...
         'Rolling Resistance'; 'Battery Capacity'; 'Cruise Speed'};

effects = [riderChange; CdAChange; massChange; CrrChange; ...
           batteryChange; speedChange];

T = table(names,effects, ...
    'VariableNames',{'Parameter','EffectMagnitude'});

disp(' ');
disp('==============================');
disp('Sensitivity Analysis');
disp('==============================');
disp(T);
end


%% ============================================================
function optimizationReport(studies)
% Prints the best result from each study.

fprintf('\n===============================================\n');
fprintf('        eHPVC PERFORMANCE SUMMARY\n');
fprintf('===============================================\n\n');

[bestSpeed,idx] = max(studies.rider.avgSpeed);
fprintf('RIDER POWER STUDY\n');
fprintf('Best Average Speed : %.2f mph\n',bestSpeed);
fprintf('Best Rider Power   : %.0f W\n',studies.rider.x(idx));
fprintf('Battery Remaining  : %.2f %%\n\n',studies.rider.SOC(idx));

[bestSpeed,idx] = max(studies.CdA.avgSpeed);
fprintf('AERODYNAMIC STUDY\n');
fprintf('Best Average Speed : %.2f mph\n',bestSpeed);
fprintf('Best CdA           : %.3f\n',studies.CdA.x(idx));
fprintf('Battery Remaining  : %.2f %%\n\n',studies.CdA.SOC(idx));

[bestSpeed,idx] = max(studies.mass.avgSpeed);
fprintf('MASS STUDY\n');
fprintf('Best Average Speed : %.2f mph\n',bestSpeed);
fprintf('Best Bike Mass     : %.1f kg\n',studies.mass.x(idx));
fprintf('Battery Remaining  : %.2f %%\n\n',studies.mass.SOC(idx));

[bestSpeed,idx] = max(studies.Crr.avgSpeed);
fprintf('ROLLING RESISTANCE STUDY\n');
fprintf('Best Average Speed : %.2f mph\n',bestSpeed);
fprintf('Best Crr           : %.4f\n',studies.Crr.x(idx));
fprintf('Battery Remaining  : %.2f %%\n\n',studies.Crr.SOC(idx));

[bestSpeed,idx] = max(studies.speed.avgSpeed);
fprintf('SPEED STRATEGY STUDY\n');
fprintf('Best Average Speed : %.2f mph\n',bestSpeed);
fprintf('Cruise Speed       : %.1f mph\n',studies.speed.x(idx));
fprintf('Battery Remaining  : %.2f %%\n\n',studies.speed.SOC(idx));

[maxSOC,idx] = max(studies.battery.SOC);
fprintf('BATTERY STUDY\n');
fprintf('Best Capacity      : %.0f Wh\n',studies.battery.x(idx));
fprintf('Remaining Battery  : %.2f %%\n',maxSOC);
fprintf('Distance           : %.2f miles\n\n',studies.battery.distance(idx));

fprintf('===============================================\n');
fprintf('End of Optimization Report\n');
fprintf('===============================================\n');
end


%% ============================================================
function exportResults(results, studies)
% Exports simulation and parameter-study data to CSV files.

simulationTable = table( ...
    results.time(:), results.speed(:), results.targetSpeed(:), ...
    results.acceleration(:), results.distance(:), ...
    results.motorPower(:), results.motorTorque(:), ...
    results.riderPower(:), results.batteryPower(:), ...
    results.batteryCurrent(:), results.batteryVoltage(:), ...
    results.SOC(:), ...
    'VariableNames',{'Time_s','Speed_mps','TargetSpeed_mps', ...
    'Acceleration','Distance_m','MotorPower_W','MotorTorque_Nm', ...
    'RiderPower_W','BatteryPower_W','BatteryCurrent_A', ...
    'BatteryVoltage_V','SOC_percent'});

writetable(simulationTable,'SimulationResults.csv');

writeStudy(studies.rider,'RiderPower', 'RiderStudy.csv');
writeStudy(studies.CdA,'CdA', 'CdAStudy.csv');
writeStudy(studies.mass,'Mass', 'MassStudy.csv');
writeStudy(studies.Crr,'RollingResistance', 'RollingResistanceStudy.csv');
writeStudy(studies.speed,'CruiseSpeed', 'SpeedStudy.csv');
writeStudy(studies.battery,'BatteryCapacity', 'BatteryStudy.csv');

fprintf('CSV export complete.\n');
end


%% ============================================================
function writeStudy(study, xName, fileName)
% Helper function for CSV export.

T = table(study.x(:), study.avgSpeed(:), study.energy(:), ...
          study.SOC(:), study.distance(:), ...
    'VariableNames',{xName,'AverageSpeed','EnergyUsed','SOC','Distance'});

writetable(T,fileName);
end


%% ============================================================
function race = lapSimulator(P,numLaps)
% Optional multi-lap simulator.
% Note: each lap currently starts with a full battery because
% simulateRace initializes a new battery for every lap.

if nargin < 2
    numLaps = 1;
end

totalDistance = 0;
totalTime = 0;
totalEnergy = 0;

lapTime = zeros(1,numLaps);
lapDistance = zeros(1,numLaps);
lapEnergy = zeros(1,numLaps);
lapSOC = zeros(1,numLaps);

for lap = 1:numLaps
    fprintf('Running Lap %d of %d...\n',lap,numLaps);

    R = simulateRace(P);

    lapTime(lap) = R.time(end);
    lapDistance(lap) = R.distanceMiles;
    lapEnergy(lap) = R.energyUsedWh;
    lapSOC(lap) = R.remainingSOC;

    totalDistance = totalDistance + R.distanceMiles;
    totalTime = totalTime + R.time(end);
    totalEnergy = totalEnergy + R.energyUsedWh;
end

valid = lapTime > 0;

race.totalDistance = totalDistance;
race.totalTime = totalTime;
race.totalEnergy = totalEnergy;
race.averageLapTime = mean(lapTime(valid));
race.averageSpeed = totalDistance/(totalTime/3600);
race.finalSOC = lapSOC(find(valid,1,'last'));

race.lapTime = lapTime;
race.lapDistance = lapDistance;
race.lapEnergy = lapEnergy;
race.lapSOC = lapSOC;
end
