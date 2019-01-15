function testDMPSequencing()

set_matlab_utils_path();

clear_all();

load('data/dmp_data.mat', 'DMP_data');

a_s0 = 5.0;

y0 = [0.0; 0.1];
g1 = [0.59; 0.9];
g2 = [0.85; 0.6];
T = 5;

dmp1 = DMP_data{1};
dmp2 = DMP_data{2};

%% set initial values
can_clock_ptr = dmp1{1}.can_clock_ptr;
Dim = length(dmp1);

t = 0.0;
x = 0.0;
dt = 0.002;

iters = 1;
Time = [];

y = y0;
dy = zeros(Dim,1);
ddy = zeros(Dim,1);
Y_data = [];
dY_data = [];
ddY_data = [];

dmp2a = cell(Dim,1);
y2a = y;
dy2a = dy;
ddy2a = ddy;
Y2a_data = [];
dY2a_data = [];
ddY2a_data = [];

y3 = y;
dy3 = dy;
ddy3 = ddy;
g3 = y3;
h3 = 120;
Y3_data = [];
dY3_data = [];
ddY3_data = [];
G3_data = [];

a_s = a_s0;
ys = y;
dys = dy;
ddys = ddy;
dddys = zeros(Dim,1);
Ys_data = [];
dYs_data = [];
ddYs_data = [];

trans_ind = [];

t_end = T;
tau = t_end;
dmp = [];

trans_timestamps = [0 0.4 0.7]*T;
goal_trans = [g1 g2 g1];
i_trans = 1;

%% simulate
while (true)
    
    if ( (i_trans<=length(trans_timestamps)) && ...
         ( (abs(t-trans_timestamps(i_trans))<dt/2) || ( abs(t-trans_timestamps(i_trans))==dt/2 && t>trans_timestamps(i_trans)) ) )
     
        % move to next goal
        g = goal_trans(:,i_trans);

        if (g(2)/g(1) > 1.0)
            dmp = dmp1;
        else
            dmp = dmp2;
        end

        % reset init conditions
        y0 = y;
        x = 0.0; 
        t_rem = T - t;
        tau = t_rem;
        
        % switch for DMP-filt
        a_s = a_s0;

        % switch for DMP3
        for i=1:Dim
            g3(i) = ( tau^2*ddy(i) + dmp{i}.a_z*tau*dy(i) - dmp{i}.shapeAttractor(x, y0(i), g(i)) )/(dmp{i}.a_z*dmp{i}.b_z) + y(i);
        end
        
        % switch for DMP2a
        for i=1:Dim
            dmp2a{i} = dmp{i}.copy();
            psi = dmp2a{i}.kernelFunction(x);
            psi = psi/sum(psi);
            psi = dmp2a{i}.shapeAttrGating(x)*dmp2a{i}.forcingTermScaling(y0(i),g(i)) * psi;
            f = dmp2a{i}.shapeAttractor(x, y0(i), g(i));
            fd = tau^2*ddy(i) + tau*dmp2a{i}.a_z*dy(i) - dmp2a{i}.a_z*dmp2a{i}.b_z*(g(i)-y(i));
            dw = pinv(psi) * ( fd - f );
            dmp2a{i}.w = dmp2a{i}.w + dw(:);
        end
        
        trans_ind = [trans_ind iters];
        i_trans = i_trans + 1;
    end
    
    can_clock_ptr.setTau(tau);
        
    %% data logging
    Time = [Time t];
    
    % log DMP2 data
    Y_data = [Y_data y];
    dY_data = [dY_data dy];  
    ddY_data = [ddY_data ddy];
    
    % log DMP-filt data
    Ys_data = [Ys_data ys];
    dYs_data = [dYs_data dys];  
    ddYs_data = [ddYs_data ddys];
    
    % log DMP3 data
    Y3_data = [Y3_data y3];
    dY3_data = [dY3_data dy3];  
    ddY3_data = [ddY3_data ddy3];
    G3_data = [G3_data g3];
    
    % log DMP2a data
    Y2a_data = [Y2a_data y2a];
    dY2a_data = [dY2a_data dy2a];  
    ddY2a_data = [ddY2a_data ddy2a];

    %% DMP2 simulation
    for i=1:Dim
        ddy(i) = dmp{i}.getAccel(y(i), dy(i), y0(i), 0.0, 0.0, x, g(i), tau);
    end
    
    %% DMP2a simulation
    for i=1:Dim
        ddy2a(i) = dmp2a{i}.getAccel(y2a(i), dy2a(i), y0(i), 0.0, 0.0, x, g(i), tau);
    end

    %% DMP3 simulation
    for i=1:Dim
        ddy3(i) = ( dmp{i}.goalAttractor(x, y3(i), tau*dy3(i), g3(i)) + dmp{i}.shapeAttractor(x, y0(i), g(i)) )/tau^2;
    end
    dg3 = (h3/tau) * (g - g3);
    
    %% DMP-filt simulation
    Ms = 3*a_s*eye(Dim,Dim);
    Ds = 3*a_s^2*eye(Dim,Dim);
    Ks = a_s^3*eye(Dim,Dim);
    dddys = Ms*(ddy - ddys) + Ds*(dy - dys) + Ks*(y - ys);
    
    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria
    if (t>=t_end && norm(ys-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    
    y = y + dy*dt;
    dy = dy + ddy*dt;

    y3 = y3 + dy3*dt;
    dy3 = dy3 + ddy3*dt;
    g3 = g3 + dg3*dt;
    
    y2a = y2a + dy2a*dt;
    dy2a = dy2a + ddy2a*dt;
    
    ys = ys + dys*dt;
    dys = dys + ddys*dt;
    ddys = ddys + dddys*dt;
    if (a_s < 200)
        da_s = 5*a_s;
        a_s = a_s + da_s*dt;
    end

end

%% calculate L2 error norm
es_error = sqrt(sum((Y_data-Ys_data).^2,1));
e3_error = sqrt(sum((Y_data-Y3_data).^2,1));
e2a_error = sqrt(sum((Y_data-Y2a_data).^2,1));

%% calculate jerk
dddY_data = zeros(Dim,length(Time));
dddYs_data = zeros(size(dddY_data));
dddY3_data = zeros(size(dddY_data));
dddY2a_data = zeros(size(dddY_data));
for i=1:Dim
    dddYs_data(i,:) = diff([ddYs_data(i,1) ddYs_data(i,:)])/dt;
    dddY3_data(i,:) = diff([ddY3_data(i,1) ddY3_data(i,:)])/dt;
    dddY2a_data(i,:) = diff([ddY2a_data(i,1) ddY2a_data(i,:)])/dt;
    dddY_data(i,:) = diff([ddY_data(i,1) ddY_data(i,:)])/dt;
end
dddYs_norm = sqrt(sum((dddYs_data).^2,1));
dddY3_norm = sqrt(sum((dddY3_data).^2,1));
dddY2a_norm = sqrt(sum((dddY2a_data).^2,1));
dddY_norm = sqrt(sum((dddY_data).^2,1));

trans_points = Y_data(:,trans_ind);

%% Print aggregate results
N_data = length(Time);
results = {'       ', 'DMP2', 'DMP-filt', 'DMP3', 'DMP2a';
           'L2 norm', '-', sum(es_error)*dt, sum(e3_error)*dt, sum(e2a_error)*dt;
           'jerk   ', sum(dddY_norm)*dt, sum(dddYs_norm)*dt, sum(dddY3_norm)*dt, sum(dddY2a_norm)*dt;
          };
      
 display(results);

%% Plot results
Data = {struct('Time',Time, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data); ...
        struct('Time',Time, 'Y',Ys_data, 'dY',dYs_data, 'ddY',ddYs_data); ...
        struct('Time',Time, 'Y',Y3_data, 'dY',dY3_data, 'ddY',ddY3_data); ...
        struct('Time',Time, 'Y',Y2a_data, 'dY',dY2a_data, 'ddY',ddY2a_data)};
data_labels = {'DMP2', 'DMP-filt', 'DMP3', 'DMP2a'};
line_styles = {'-', '--', '-.', ':'};
colors = {[1.0 0.0 0.8]; [0 0 0.85]; [0 0.85 0]; [0.83 0.59 0.09]};
plotResults(Time, Data, data_labels, line_styles, colors, trans_points, goal_trans);

fig = figure;
ax = axes('Parent',fig);
hold(ax, 'on');
plot(Time, es_error, 'LineWidth',1.5, 'Color',[0 0 0.85], 'LineStyle','-');
plot(Time, e3_error, 'LineWidth',1.5, 'Color',[0.85 0.33 0.1], 'LineStyle','--');
plot(Time, e2a_error, 'LineWidth',1.5, 'Color',[0.93 0.69 0.13], 'LineStyle',':');
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
ylabel('L2 norm [$m$]', 'interpreter','latex', 'fontsize',15);
title('L2 error norm from DMP2', 'interpreter','latex', 'fontsize',15);
legend({'DMP-filt','DMP3','DMP2a'}, 'interpreter','latex', 'fontsize',15);
axis(ax,'tight');

fig = figure;
ax = axes('Parent',fig);
hold(ax, 'on');
% plot(Time, dddY_norm, 'LineWidth',1.5, 'Color',colors{1}, 'LineStyle',line_styles{1});
plot(Time, dddYs_norm, 'LineWidth',1.5, 'Color',[0 0 0.85], 'LineStyle','-');
plot(Time, dddY3_norm, 'LineWidth',1.5, 'Color',[0.85 0.33 0.1], 'LineStyle','--');
plot(Time, dddY2a_norm, 'LineWidth',1.5, 'Color',[0.93 0.69 0.13], 'LineStyle',':');
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
ylabel('jerk [$m/s^3$]', 'interpreter','latex', 'fontsize',15);
title('Jerk', 'interpreter','latex', 'fontsize',15);
legend({'DMP-filt','DMP3','DMP2a'}, 'interpreter','latex', 'fontsize',15);
axis(ax,'tight');

% figure;
% for i=1:Dim
%     subplot(Dim,1,i);
%     plot(Time, G3_data(i,:), 'LineWidth',1.5);
%     if (i==Dim), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14); end
%     ylabel(['Dim $' num2str(i) '$'], 'interpreter','latex', 'fontsize',14);
%     if (i==1), title('Goal evolution', 'interpreter','latex', 'fontsize',14); end
%     axis tight;
% end

end

function plotResults(Time, Data, data_labels, line_styles, colors, trans_points, goals)

if (~iscell(Data)), Data = {Data}; end

n_data = length(Data);

Dim = size(Data{1}.Y,1);
Y0 = Data{1}.Y(:,1);

fontsize = 15;

fig1 = figure('Position',[150 250 1200 600]);
ax1 = cell(3,2);
ax1_ylabels = {'position [$m$]', 'velocity [$m/s$]', 'acceleration [$m/s^2$]'};
for i=1:size(ax1,1)
    for j=1:size(ax1,2)
        ax1{i,j} = subplot(3,3,(i-1)*3+j+1, 'Parent',fig1);
        axis(ax1{i,j},'tight');
        hold(ax1{i,j},'on');
        xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',ax1{i,j});
        ylabel(ax1_ylabels{i},'interpreter','latex','fontsize',fontsize, 'Parent',ax1{i,j});
    end
end
for n=1:n_data
    for i=1:Dim
        plot(Time, Data{n}.Y(i,:), 'LineWidth',1.5, 'LineStyle',line_styles{n}, 'Color',colors{n}, 'Parent',ax1{1,i});
        plot(Time, Data{n}.dY(i,:), 'LineWidth',1.5, 'LineStyle',line_styles{n}, 'Color',colors{n}, 'Parent',ax1{2,i});
        plot(Time, Data{n}.ddY(i,:), 'LineWidth',1.5, 'LineStyle',line_styles{n}, 'Color',colors{n}, 'Parent',ax1{3,i});
    end
end
for i=1:Dim
    title(['Dim ' num2str(i)],'interpreter','latex','fontsize',fontsize, 'Parent',ax1{1,i});
end
legend(ax1{1,1}, data_labels,'interpreter','latex','fontsize',fontsize, 'Location','northwest','orientation','vertical');

% fig2 = figure('NumberTitle','off', 'Name','2D path', 'Position',[100 250 850 700]);
% ax2 = axes('Parent',fig2);
ax2 = subplot(3,3,[1 4 7], 'Parent',fig1);
axis(ax2,'tight');
xlabel('$X$ [$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax2);
ylabel('$Y$ [$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax2);
hold(ax2,'on');
for n=1:n_data
    plot(Data{n}.Y(1,:), Data{n}.Y(2,:), 'LineWidth',1.5, 'LineStyle',line_styles{n}, 'Color',colors{n}, 'Parent',ax2);
end
plot([-0.1 1.0], [-0.1 1.0], 'LineWidth',1.5, 'Color',[0.75 0.75 0.75], 'LineStyle','--', 'Parent',ax2);
scatter(Y0(1), Y0(2), 'MarkerEdgeColor','green', 'LineWidth',2.5, 'Marker','o', 'SizeData',120, 'Parent',ax2);
for j=1:size(goals,2)
    scatter(goals(1,j), goals(2,j), 'LineWidth',2.5, 'Marker','*', 'SizeData',120, 'Parent',ax2); %, 'MarkerEdgeColor',[0.6 0.2 0.0], 'MarkerFaceColor',[0.6 0.2 0.0]
end
trans_points = trans_points(:,2:end);
scatter(trans_points(1,:), trans_points(2,:), 'LineWidth',2.5, 'Marker','+', 'SizeData',120, 'MarkerEdgeColor',[1 0 1], 'MarkerFaceColor',[1 0 1], 'Parent',ax2);
legend_labels = [{data_labels{:}}, 'dmp switch bound','start','goal 1','goal 2','transition points'];
legend(ax2, legend_labels,'interpreter','latex','fontsize',fontsize, 'Location','northwest');
 
end