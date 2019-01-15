function trainAndSimulateDMP()

set_matlab_utils_path();

load('data/training_data.mat','Data');

N_kernels = 15;
a_z = 16;
b_z = a_z/4;
train_method = 'LS';
dt = 0.005;

can_clock_ptr = CanonicalClock();

N = length(Data);
DMP_data = cell(N,1);

for n=1:N
    
    %% Train the DMP
    disp('DMP training...');
    [dmp, offline_train_mse] = trainDMP(Data{n}, N_kernels, a_z, b_z, train_method, can_clock_ptr);
    DMP_data{n} = dmp;
    
    %% DMP simulation
    disp('DMP simulation...');
    y0 = Data{n}.Y(:,1);
    g = Data{n}.Y(:,end);
    t_end = Data{n}.Time(:,end);
    [Time, Y_data, dY_data, ddY_data] = simulateDMP(dmp, y0, g, t_end, dt);

    %% plot simulation results
    plotSimResults(Time, Y_data, dY_data, ddY_data, Data{n}.Time, Data{n}.Y, Data{n}.dY, Data{n}.ddY);

end

save('data/dmp_data.mat', 'DMP_data');

end

function [dmp, offline_train_mse] = trainDMP(Data, N_kernels, a_z, b_z, train_method, can_clock_ptr)

    Timed = Data.Time;
    yd_data = Data.Y;
    dyd_data = Data.dY;
    ddyd_data = Data.ddY;
    
    Dim = size(yd_data,1);

    dmp = cell(Dim,1);
    
    for i=1:Dim
        shapeAttrGatingPtr = SigmoidGatingFunction(1.0, 0.97);
        % shapeAttrGatingPtr = LinGatingFunction(1.0, 0.0);
        % shapeAttrGatingPtr = ExpGatingFunction(1.0, 0.05);
        dmp{i} = DMP(N_kernels, a_z, b_z, can_clock_ptr, shapeAttrGatingPtr);
    end

    offline_train_mse = zeros(Dim,1); 
    for i=1:Dim
        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(train_method, Timed, yd_data(i,:), dyd_data(i,:), ddyd_data(i,:));      
    end
    
end

function [Time, Y_data, dY_data, ddY_data] = simulateDMP(dmp, y0, g, T, dt)
%% Simulates a dmp
% @param[in] dmp: Dim x 1 cell array, where each cell is a 1D DMP.
% @param[in] y0: Dim x 1 vector with the initial position..
% @param[in] g: Dim x 1 vector with the goal/target position.
% @param[in] T: Movement total time duration.
% @param[in] dt: Simulation timestep.
% @param[out] Time: 1 x N rowvector with simulation output timestamps.
% @param[out] Y_data: Dim x N matrix with simulation output positions.
% @param[out] dY_data: Dim x N matrix with simulation output velocities.
% @param[out] ddY_data: Dim x N matrix with simulation output accelerations.
%


%% set initial values
can_clock_ptr = dmp{1}.can_clock_ptr;
Dim = length(dmp);
x = 0.0;
dx = 0.0;
ddy = zeros(Dim,1);
dy = zeros(Dim,1);
y = y0;
t = 0.0;
dz = zeros(Dim,1);
z = zeros(Dim,1);

t_end = T;
can_clock_ptr.setTau(t_end);

iters = 0;
Time = [];
Y_data = [];
dY_data = [];
ddY_data = [];
x_data = [];

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Y_data = [Y_data y];
    dY_data = [dY_data dy];  
    ddY_data = [ddY_data ddy];
    % x_data = [x_data x];

    %% DMP simulation
    for i=1:Dim
        y_c = 0.0;
        z_c = 0.0;
        [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);
        ddy(i) = dz(i)/dmp{i}.getTau();
    end

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria
    if (t>=t_end) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;

end


end

function plotSimResults(Time, Y_data, dY_data, ddY_data, Timed, Yd_data, dYd_data, ddYd_data)

    Dim = size(Y_data,1);
    
    fontsize = 14;
    % figure('NumberTitle', 'off', 'Name', ['Demo ' num2str(n)]);
    figure;
    k = 1;
    for i=1:Dim
        subplot(Dim,3,k);
        plot(Time,Y_data(i,:), Timed,Yd_data(i,:));
        if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
        if (i==Dim), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        legend('dmp','demo');
        axis tight;
        subplot(Dim,3,k+1);
        plot(Time,dY_data(i,:), Timed,dYd_data(i,:));
        if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
        if (i==Dim), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        axis tight;
        subplot(Dim,3,k+2);
        plot(Time,ddY_data(i,:), Timed,ddYd_data(i,:));
        if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
        if (i==Dim), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        axis tight;
        k = k+3;
    end

end