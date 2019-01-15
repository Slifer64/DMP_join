function createTrainingData()

Ts = 0.002;
t_end = 5;

Time = 0:Ts:t_end;

y0 = [0.1; 0.2];
g = y0 + [0.79; 0.9];

Y_data1 = y0 + (g-y0)*(10*(Time/t_end).^3 - 15*(Time/t_end).^4 + 6*(Time/t_end).^5); % + 1.5*[-0.26; 0.12]*exp(-t_end/2*(Time-t_end/2.2).^2);
[dY_data1, ddY_data1] = calcDerivatives(Y_data1, Ts);
 
% x2 = Y_data1(1,:);
% x2 = (x2 - min(x2));
% x2 = x2 / abs(max(x2));
% y2 = - 4.0 * sqrt( 1 - (x2/1).^2 );
% Y_data2 = [x2; y2];
Y_data2 = y0 + (g-y0)*(10*(Time/t_end).^3 - 15*(Time/t_end).^4 + 6*(Time/t_end).^5) - 1.7*[-0.15; 0.1]*exp(-(Time-t_end/2).^2 / (2 * (t_end/10)^2 ) );
[dY_data2, ddY_data2] = calcDerivatives(Y_data2, Ts);

Data = cell(2,1);
Data{1} = struct('Time',Time, 'Y',Y_data1, 'dY',dY_data1, 'ddY',ddY_data1);
Data{2} = struct('Time',Time, 'Y',Y_data2, 'dY',dY_data2, 'ddY',ddY_data2);
plotData(Data, 'legend',{'MP1','MP2'});

save('data/training_data.mat','Data');

end

function [dY_data, ddY_data] = calcDerivatives(Y_data, Ts)

dY_data = zeros(size(Y_data));
ddY_data = zeros(size(Y_data));
for i=1:size(Y_data,1)
    dY_data(i,:) = diff([Y_data(i,1) Y_data(i,:)])/Ts;
    ddY_data(i,:) = diff([dY_data(i,1) dY_data(i,:)])/Ts;
end

end