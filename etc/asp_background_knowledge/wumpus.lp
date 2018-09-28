%definitions
heading(up) :- heading(0).
heading(left) :- heading(1). 
heading(down) :- heading(2).
heading(right) :- heading(3).

#const n = 6.
timestep(0..n-1).
heading(0..3).

fieldSize(4).
coord(1..X) :- fieldSize(X).
field(X,Y) :- coord(X), coord(Y).

%initial position
holds(on(1,1),0).
%on(0,0).
holds(heading(right),0).
%occurs(move,0).
%occurs(0,1).


%known actions
action(move).
action(turnLeft).
action(turnRight).
action(pickup).
action(shoot).

%gold

nextAction(pickUp) :- field(X,Y) , on(X,Y), glitter(X,Y).
nextAction(leave) :- gold(agent), on(initialX(X), initialY(Y)).

%unless a moveTo-Goal is given, just continue to move in current direction
nextAction(move) :- 

%define field ahead of agent
fieldAhead(X+1,Y) :- field(X,Y), field(X+1,Y), heading(down).
fieldAhead(X-1,Y) :- field(X,Y), field(X-1,Y), heading(up).
fieldAhead(X,Y+1) :- field(X,Y), field(X,Y+1), heading(right).
fieldAhead(X,Y-1) :- field(X,Y), field(X,Y-1), heading(left).

%define field adjacency
fieldAdjacent(X,Y,X+1,Y) :- field(X,Y), field(X+1,Y).
fieldAdjacent(X,Y,X-1,Y) :- field(X,Y), field(X-1,Y)
fieldAdjacent(X,Y,X,Y+1) :- field(X,Y), field(X,Y+1).
fieldAdjacent(X,Y,X,Y-1) :- field(X,Y), field(X,Y-1).



%position/heading of agent on a field, can be changed by movement
% fluent(on(X,Y)) :- field(X,Y).
%fluent(heading(X)) :- heading(X).

%%occurence of actions general
%occurs(A,T) :- action(A), timestep(T), not -occurs(A,T).
%%CWA
%%-occurs(A,T) :- not occurs(A,T), action(A), timestep(T).
%%not more than 1 action at a given timestep
-occurs(B,T) :- occurs(A,T), timestep(T), action(A), action(B), A != B.

%% for all timesteps, each varying relation is defined
%holds(on(X,Y),T) :- timestep(T), not -holds(on(X,Y),T), field(X,Y).
%%CWA
%%-holds(F,T) :- not holds(F,T), fluent(F), timestep(T).
%-holds(on(X,Y),T) :- not holds(on(X,Y),T), field(X,Y), timestep(T).
%%inertia
holds(on(X,Y),T+1) :- holds(on(X,Y),T), not -holds(on(X,Y),T+1), T < n.
-holds(on(X,Y),T+1) :- -holds(on(X,Y),T), not holds(on(X,Y),T+1), T < n.
holds(heading(X),T+1) :- holds(heading(X),T), not -holds(heading(X),T+1), T < n.
-holds(heading(X),T+1) :- -holds(heading(X),T), not holds(heading(X),T+1), T < n.

%% only allow one state of being
%:- holds(on(A,B), T), holds(on(E,F), T), A != E, field(A,B), field(E,F).
%:- holds(on(A,B), T), holds(on(E,F), T), B != F, field(A,B), field(E,F).
%:- holds(heading(X),T), holds(heading(Y),T), X != Y, heading(X), heading(Y).

%moving in direction of current heading (right, left, up, down)
holds(on(X,Y+1),T+1) :- occurs(move,T), holds(heading(right),T), field(X,Y), field(X,Y+1), holds(on(X,Y),T), T < n. 
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(right),T), field(X,Y), field(X,Y+1), holds(on(X,Y),T), T < n. 

holds(on(X,Y-1),T+1) :- occurs(move,T), holds(heading(left),T), field(X,Y), field(X,Y-1), holds(on(X,Y),T), T < n.
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(left),T), field(X,Y), field(X,Y-1), holds(on(X,Y),T), T < n.

holds(on(X+1,Y),T+1) :- occurs(move,T), holds(heading(up),T), field(X,Y), field(X+1,Y), holds(on(X,Y),T), T < n.
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(up),T), field(X,Y), field(X+1,Y), holds(on(X,Y),T), T < n.

holds(on(X-1,Y),T+1) :- occurs(move,T), holds(heading(down),T), field(X,Y), field(X-1,Y), holds(on(X,Y),T), T < n.
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(down),T), field(X,Y), field(X-1,Y), holds(on(X,Y),T), T < n.


%turning according to current heading
holds(heading(left),T+1) :- holds(heading(up),T), timestep(T), occurs(turnLeft,T), T < n.
-holds(heading(up),T+1) :- holds(heading(up),T), timestep(T), occurs(turnLeft,T), T < n.

holds(heading(up),T+1) :- holds(heading(right),T), timestep(T) ,occurs(turnLeft,T), T < n.
-holds(heading(right),T+1) :- holds(heading(right),T), timestep(T) ,occurs(turnLeft,T), T < n.

holds(heading(down),T+1) :- holds(heading(left),T), timestep(T) ,occurs(turnLeft,T), T < n.
-holds(heading(left),T+1) :- holds(heading(left),T), timestep(T) ,occurs(turnLeft,T), T < n.

holds(heading(right),T+1) :- holds(heading(down),T), timestep(T) ,occurs(turnLeft,T), T < n.
-holds(heading(down),T+1) :- holds(heading(down),T), timestep(T) ,occurs(turnLeft,T), T < n.



holds(heading(right),T+1) :- holds(heading(up),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(up),T+1) :- holds(heading(up),T), timestep(T), occurs(turnRight,T), T < n.

holds(heading(down),T+1) :- holds(heading(right),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(right),T+1) :- holds(heading(right),T), timestep(T), occurs(turnRight,T), T < n.

holds(heading(up),T+1) :- holds(heading(left),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(left),T+1) :- holds(heading(left),T), timestep(T), occurs(turnRight,T), T < n.

holds(heading(left),T+1) :- holds(heading(down),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(down),T+1) :- holds(heading(down),T), timestep(T), occurs(turnRight,T), T < n.

#show holds/2.
%%#show field/2.
#show occurs/2.

%%planning 
goal(T) :- holds(on(3,3),T).
success :- goal(T), T <=n.
:- not success.
1{occurs(A,T) : action(A)}1 :- not goal(T), timestep(T), T < n.
occurs(A,T) | -occurs(A,T) :- action(A) , timestep(T), T<n.
%%actionPerformed :- action(A), timestep(T), occurs(A,T).


%%when a field is stinky, a wumpus can be at the neighbor fields
wumpus(X+1,Y) :- fieldAdjacent(X,Y,X+1,Y), stinky(X,Y), not -wumpus(X+1,Y).
wumpus(X-1,Y) :- fieldAdjacent(X,Y,X-1,Y), stinky(X,Y), not -wumpus(X-1,Y).
wumpus(X,Y+1) :- fieldAdjacent(X,Y,X,Y+1), stinky(X,Y) not -wumpus(X,Y+1).
wumpus(X,Y-1) :- fieldAdjacent(X,Y,X,Y-1), stinky(X,Y), not -wumpus(X,Y-1).
%%if a field is known not to be stinky, there are no adjacent wumpi right now.
-wumpus(X,Y+1) :- fieldAdjacent(X,Y,X,Y+1), -stinky(X,Y).
-wumpus(X,Y-1) :- fieldAdjacent(X,Y,X,Y-1), -stinky(X,Y).
-wumpus(X+1,Y) :- fieldAdjacent(X,Y,X+1,Y), -stinky(X,Y).
-wumpus(X-1,Y) :- fieldAdjacent(X,Y,X-1,Y), -stinky(X,Y).

%%traps, analog to wumpus
%%when a field is drafty, a trap can be at the neighbor fields
trap(X+1,Y) :- fieldAdjacent(X,Y,X+1,Y), drafty(X,Y), not -wumpus(X+1,Y).
trap(X-1,Y) :- fieldAdjacent(X,Y,X-1,Y), drafty(X,Y), not -wumpus(X-1,Y).
trap(X,Y+1) :- fieldAdjacent(X,Y,X,Y+1), drafty(X,Y), not -wumpus(X,Y+1).
trap(X,Y-1) :- fieldAdjacent(X,Y,X,Y-1), drafty(X,Y), not -wumpus(X,Y-1).
%%if a field is known not to be drafty, there are no adjacent traps.
-trap(X,Y+1) :- fieldAdjacent(X,Y,X,Y+1), -drafty(X,Y).
-trap(X,Y-1) :- fieldAdjacent(X,Y,X,Y-1), -drafty(X,Y).
-trap(X+1,Y) :- fieldAdjacent(X,Y,X+1,Y), -drafty(X,Y).
-trap(X-1,Y) :- fieldAdjacent(X,Y,X-1,Y), -drafty(X,Y).

%%TODO: traps are static, wumpi can move

%%wumpus and trap cannot be on the same field
-trap(X,Y) :- field(X,Y), wumpus(X,Y).
-wumpus(X,Y) :- field(X,Y), trap(X,Y).

%%danger 
safe(X,Y) :- field(X,Y), not wumpus(X,Y).
safe(X,Y) :- field(X,Y), not trap(X,Y).















