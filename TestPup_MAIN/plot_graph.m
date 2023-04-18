figure
hold on
p = plot(0,0 , 'o',MarkerFaceColor='red');
r = plot(0,0 , 'o',MarkerFaceColor='green');
hold off
a1 = animatedline('Color',[0 .7 .7]);
a2 = animatedline('Color',[0 .5 .5]);

axis([0 20 -1 1])
x = linspace(0,20,10000);
t = text(0,0, "sadg");
for k = 1:length(x)
    % first line
    xk = x(k);
    ysin = sin(xk);
    addpoints(a1,xk,ysin);    

    % second line
    ycos = cos(xk);
    addpoints(a2,xk,ycos);

    p.XData = xk;
    p.YData = ycos;

    r.XData = xk;
    r.YData = ysin;

    t.Position = [xk, ysin];
    t.String = ['\leftarrow' num2str(ycos)];
    
  

    % update screen
    drawnow limitrate
end
[x,y] = getpoints(a1);

% tic
% h = [];   % Handle of line object
% for thetha = linspace(0 , 10*pi , 500)
%     y = sin(x + thetha);
%     
%     if isempty(h)
%         % If Line still does not exist, create it
%         h = plot(x,y);
%     else
%         % If Line exists, update it
%         set(h , 'YData' , y)
%     end
%     drawnow
% end
% toc

% tic
% figure
% h = animatedline;
% 
% for thetha = linspace(0 , 10*pi , 500)
%     y = sin(x + thetha);
%     clearpoints(h)
%     addpoints(h , x , y)
%     drawnow limitrate
% end
toc
% x = linspace(0,8);
% y = sin(x);
% ln = plot(x,y);
% 
% ln.XDataSource = 'x';
% ln.YDataSource = 'y';
% y = sin(3.*x);
% 
% refreshdata;
