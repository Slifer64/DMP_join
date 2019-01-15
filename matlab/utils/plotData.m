function [fig1, fig2] = plotData(Data, varargin)

    if (~iscell(Data)), Data = {Data}; end
        
    N = length(Data);
    
    %% Parse the input arguments
    [legends, lineStyles] = parseInputArguments(N, varargin{:});
    
    fontsize = 14;
    linewidth = 1.5;
    
    fig1 = figure('NumberTitle', 'off', 'Name', '1D plots');
    Dim = size(Data{1}.Y,1);
    ax1 = cell(Dim,3);
    
    for i=1:Dim
        for j=1:3
            ax1{i,j} = subplot(Dim,3,(i-1)*3+j, 'Parent',fig1);
            hold(ax1{i,j},'on');
            axis(ax1{i,j},'tight');
        end
    end
    for j=1:3, xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',ax1{Dim,j}); end
    for i=1:Dim, ylabel(['dim-$' num2str(i) '$'],'interpreter','latex','fontsize',fontsize, 'Parent',ax1{i,1}); end
    title('pos [$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax1{1,1});
    title('vel [$m/s$]','interpreter','latex','fontsize',fontsize, 'Parent',ax1{1,2});
    title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize, 'Parent',ax1{1,3});
    
    fig2 = figure('NumberTitle', 'off', 'Name', '2D plot');
    ax2 = axes('Parent',fig2);
    axis(ax2,'tight');
    xlabel('$X$ [$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax2);
    ylabel('$Y$ [$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax2);
    hold(ax2,'on');
    
    for n=1:N

        Time = Data{n}.Time;
        Y_data = Data{n}.Y;
        dY_data = Data{n}.dY;
        ddY_data = Data{n}.ddY;

        for i=1:Dim
            plot(Time,Y_data(i,:), 'LineWidth',linewidth, 'LineStyle',lineStyles{n}, 'Parent',ax1{i,1});
            plot(Time,dY_data(i,:), 'LineWidth',linewidth, 'LineStyle',lineStyles{n}, 'Parent',ax1{i,2});
            plot(Time,ddY_data(i,:), 'LineWidth',linewidth, 'LineStyle',lineStyles{n}, 'Parent',ax1{i,3});
        end
        
        plot(Y_data(1,:), Y_data(2,:), 'LineWidth',linewidth, 'LineStyle',lineStyles{n}, 'Parent',ax2);

    end

    legend(ax1{1,1}, legends,'interpreter','latex','fontsize',fontsize);
    legend(ax2, legends,'interpreter','latex','fontsize',fontsize);

end

function [legends, lineStyles] = parseInputArguments(N, varargin)

    % initialize parser with the names and default values of the input arguments
    inPars = inputParser;
    
    inPars.KeepUnmatched = true;
    inPars.PartialMatching = false;
    inPars.CaseSensitive = false;
    
    inPars.addParameter('legend', {}, @(x)assert_cell_with_strings(x));
    inPars.addParameter('LineStyle', {}, @(x)assert_cell_with_strings(x));
    
    % Parse input arguments
    inPars.parse(varargin{:});
    
    unmatchedNames = fieldnames(inPars.Unmatched);
%     usingDefaults = inPars.UsingDefaults;
    
    legends = inPars.Results.legend;
    lineStyles = inPars.Results.LineStyle;
    
    if (isempty(legends))
        legends = cell(N,1);
        for i=1:N
            legends{i} = ['traj-$' num2str(i), '$'];
        end
    end
    
    if (isempty(lineStyles))
        lineStyles = cell(N,1);
        for i=1:N
            lineStyles{i} = '-';
        end
    end
    
%     if (~isempty(cellfun(@(x) strcmpi(x,'LegendFontSize'), usingDefaults)))
%         inArgs.LegendFontSize = inArgs.FontSize;
%     end
    
    if (~isempty(unmatchedNames))
        str = sprintf('parseInputArguments: Found unmatched argument names:\n');
        for i=1:length(unmatchedNames)
            str = [str sprintf('%s\n', unmatchedNames{i})];
        end
        warning('%s', str); 
    end
    
    % disp('parseInputArguments: Using defaults for:\n%s', usingDefaults{:});
    
end

function assert_cell_with_strings(x)

    assert( iscell(x) && (isempty(x) || sum(cellfun(@(y) ~ischar(y), x))==0 ), 'Input must be a cell array of strings.');

end