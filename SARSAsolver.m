clear all;clc;
%% Initailization guess
global W;W = 9200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg
global rho_air;rho_air = 1.225 ;        % density of air in kg/m3 

% Wind parameters
global Vw;Vw = 10;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

global N; N = 10; % Scenario
global T; T = 165;
global yf; yf = 0;
global hf; hf = 0;
global x0; x0 = 0;
global y0; y0 = 0;
global h0; h0 = 1000;
global V0; V0 = 100;
global gamma0; gamma0 = 0.26;
global chi0; chi0 = pi;
global chif; chif = 0;


global alphaMax; alphaMax = 25 / 180 * pi;
global muMax; muMax = 90 / 180 * pi;
global Vmin; Vmin = 56;

guess = spline_guess([0;0;1000],[-5000;0;0],5,N,true);
minimum = min(guess);
scale = abs(minimum-max(guess));

%% Initial Q matrix
Q = init_Q();
figure(1);
matrixPlot(Q)


Goal=n*n;
fprintf('Goal State is: %d',Goal)
% Creating Reward Matrix for the Maze 
% Possible actions are:
% 
% * Up     (i-n)
% * Down   (i+n)
% * Left   (i-1)
% * Right  (i+1)
% * Diagonally SE (i+n+1)
% * Diagonally SW (i+n-1)
% * Diagonally NE (i-n+1)
% * Diagonally NW (i-n-1)
% 
% Reward  is -Inf (~No reward) for any other actions. Thus any other action 
% other then above action will not occur.
%
reward=[];
for i=1:Goal
    reward=[reward;reshape(maze',1,Goal);];
end
for i=1:Goal
for j=1:Goal
    if j~=i-n && j~=i+n && j~=i-1 && j~=i+1 && j~=i+n+1 && j~=i+n-1 && j~=i-n+1 && j~=i-n-1
        reward(i,j)=-Inf;
    end
end
end
% SARSA algorithm
% * Initialize the Q-matrix.
% * Setting the goal state to be 'n*n'. 
% * Gamma=0.5 and alpha=0.5 (Selected after several runs)
% * Maximum number of iterations (for convergence)
%
q=zeros(size(reward));
gamma=0.5; alpha=0.5; maxItr=100;
% 
% * Repeat until Convergence OR Maximum Iterations
% * cs => current state
% * ns => next state
%
for i=1:maxItr
    
    % Starting from any Random state    
    cs=randi([1 length(reward)],1,1);
    
    % Repeate until Goal state is reached
    while(1)
        % Possible Actions from current state        
        actions=find(reward(cs,:)>0);
    
        % Next State due to possible actions
        ns=actions(randi([1 length(actions)]));
        
        % Possible Actions from Next State
        actions=find(reward(ns,:)>0);
            
        % q value, for action is choosen randomly from all possible actions
        randq=q(ns,actions(randi([1,length(actions)])));
        % Updation of Action-Value Function (SARSA)      
        q(cs,ns)=q(cs,ns)+alpha*(reward(cs,ns)+gamma*randq -q(cs,ns));
        % Break, if Goal state is reached
        if(cs == Goal)
            break;
        end
    
        % Else Current-state is Next-State
        cs=ns;
    end  
end
% Solving the maze i.e, finding a path (optimal) from START to GOAL
% * Starting from the first postion
%
start=1;move=0;
path=[start];
% 
% * Iterating until Goal-State is reached
%
while(move~=Goal)
    [~,move]=max(q(start,:));
    
    % Deleting chances of getting stuck in small loops    
    if ismember(move,path)
        [~,x]=sort(q(start,:),'descend');
        move=x(2); 
        if ismember(move,path)
            [~,x]=sort(q(start,:),'descend');
            move=x(3);  
        end
    end
    
    % Appending next action/move to the path
    path=[path,move]
    start=move;
end
% Solution of maze i.e, Optimal Path between START to GOAL
%
fprintf('Final Path: %s',num2str(path))
pmat=zeros(n,n);
[q, r]=quorem(sym(path),sym(n));
q=double(q+1);r=double(r);
q(r==0)=n;r(r==0)=n;
for i=1:length(q)
    pmat(q(i),r(i))=50;
end  
% Final Plot of the maze
%
figure
imagesc(pmat)
colormap(white)
for i=1:n
    for j=1:n
        if maze(i,j)==min(maze)
            text(j,i,'X','HorizontalAlignment','center')
        end
        if pmat(i,j)==50
            text(j,i,'\bullet','Color','red','FontSize',28)
        end
    end
end
text(1,1,'START','HorizontalAlignment','right')
text(n,n,'GOAL','HorizontalAlignment','right')
hold on
imagesc(maze,'AlphaData',0.2)
colormap(winter)
hold off
axis off
%
% created by- Bhartendu, Machine Learning & Computing