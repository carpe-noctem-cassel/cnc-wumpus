%%definitions
#program wumpusBackgroundKnowledgeFile.

heading(up) :- heading(0).
heading(left) :- heading(1). 
heading(down) :- heading(2).
heading(right) :- heading(3).


%%this encoding of the coordinates corresponds to the wumpus simulator
coord(0..X-1) :- query0(fieldSize(X)).
on(X,Y) :- query1(on(X,Y)).
heading(X) :- query1(heading(X)).
field(X,Y) :- coord(X), coord(Y).

%%TODO 

%known actions
action(move).
action(turnLeft).
action(turnRight).
action(pickup).
action(shoot).

%define field ahead of agent
fieldAhead(X+1,Y) :- field(X,Y), field(X+1,Y), heading(up), on(X,Y).
fieldAhead(X-1,Y) :- field(X,Y), field(X-1,Y), heading(down), on(X,Y).
fieldAhead(X,Y+1) :- field(X,Y), field(X,Y+1), heading(right), on(X,Y).
fieldAhead(X,Y-1) :- field(X,Y), field(X,Y-1), heading(left), on(X,Y).

%experimental: define field left/right of agent 
leftOfMe(X+1,Y) :- field(X,Y), field(X+1,Y), heading(down), on(X,Y).
leftOfMe(X-1,Y) :- field(X,Y), field(X-1,Y), heading(up), on(X,Y).
leftOfMe(X,Y+1) :- field(X,Y), field(X,Y+1), heading(left), on(X,Y).
leftOfMe(X,Y-1) :- field(X,Y), field(X,Y-1), heading(right), on(X,Y).

rightOfMe(X+1,Y) :- field(X,Y), field(X+1,Y), heading(up), on(X,Y).
rightOfMe(X-1,Y) :- field(X,Y), field(X-1,Y), heading(down), on(X,Y).
rightOfMe(X,Y+1) :- field(X,Y), field(X,Y+1), heading(right), on(X,Y).
rightOfMe(X,Y-1) :- field(X,Y), field(X,Y-1), heading(left), on(X,Y).

%%define sides being blocked
leftBlocked :- leftOfMe(X,Y), not safe(X,Y).
rightBlocked :- rightOfMe(X,Y), not safe(X,Y).

%define field adjacency
fieldAdjacent(X,Y,X+1,Y) :- field(X,Y), field(X+1,Y).
fieldAdjacent(X,Y,X-1,Y) :- field(X,Y), field(X-1,Y).
fieldAdjacent(X,Y,X,Y+1) :- field(X,Y), field(X,Y+1).
fieldAdjacent(X,Y,X,Y-1) :- field(X,Y), field(X,Y-1).

%%when a field is stinky, a wumpus can be at the neighbor fields
wumpus(X,Y) :- fieldAdjacent(A,B,X,Y), stinky(A,B), not -wumpus(X,Y).
%%if a field is known not to be stinky, there are no adjacent wumpi right now.
-wumpus(X,Y) :- fieldAdjacent(A,B,X,Y), -stinky(A,B).

%%traps, analog to wumpus
%%when a field is drafty, a trap can be at the neighbor fields
trap(X,Y) :- fieldAdjacent(A,B,X+1,Y), drafty(A,B), not -wumpus(X,Y).
%%if a field is known not to be drafty, there are no adjacent traps.
-trap(X,Y) :- fieldAdjacent(A,B,X,Y+1), -drafty(A,B).

%%wumpus and trap cannot be on the same field
-trap(X,Y) :- field(X,Y), wumpus(X,Y).
-wumpus(X,Y) :- field(X,Y), trap(X,Y).

%%danger 
safe(X,Y) :- field(X,Y), not wumpus(X,Y).
safe(X,Y) :- field(X,Y), not trap(X,Y).


%gold
nextAction(pickUp) :- field(X,Y) , on(X,Y), glitter(X,Y).
%leaving - this should trigger the planning module
nextAction(leave) :- gold(agent), on(initialX(X), initialY(Y)).

%unless a goal is given, just continue to move in current direction
nextAction(move) :- fieldAhead(A,B), safe(A,B).

%turning to free side, this might be too simple
nextAction(turnLeft) :- not nextAction(move), not leftBlocked.
nextAction(turnRight) :- not nextAction(move), not rightBlocked.

%experimental
nextAction(turnLeft) | nextAction(turnRight) :- not nextAction(move), leftBlocked, rightBlocked.

%nextAction(shoot) :- haveArrow && wumpus probably on own line wrt heading && wumpus blocking the way


%%TESTING PURPOSES:

%on(0,0).
%heading(right).

%#show nextAction/1. 
















