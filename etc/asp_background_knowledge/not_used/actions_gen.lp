%#program actions_gen(n).

%test stuff

field(0..3,0..3).
on(0,0).
heading(3).

%init
holds(on(X,Y),0) :- on(X,Y).
holds(heading(X),0) :- heading(X).

%generate steps
timestep(0..n).

%available actions
action(move).
action(turnLeft).
action(turnRight).


%how state changes by occurence of actions

fieldAhead(X-1,Y,T) :- field(X,Y), field(X-1,Y), holds(heading(0),T), holds(on(X,Y),T), timestep(T).
fieldAhead(X+1,Y,T) :- field(X,Y), field(X+1,Y), holds(heading(2),T), holds(on(X,Y),T), timestep(T).
fieldAhead(X,Y+1,T) :- field(X,Y), field(X,Y+1), holds(heading(3),T), holds(on(X,Y),T), timestep(T).
fieldAhead(X,Y-1,T) :- field(X,Y), field(X,Y-1), holds(heading(1),T), holds(on(X,Y),T), timestep(T).


holds(on(X,Y),T) :- occurs(move,T-1), fieldAhead(X,Y,T-1), timestep(T).
holds(on(X,Y),T) :- holds(on(X,Y),T-1), not occurs(move,T-1), timestep(T).
holds(heading(X),T) :- holds(heading(X),T-1), not occurs(turnLeft,T-1), not occurs(turnRight,T-1), timestep(T).
holds(heading((X+1)\4),T) :- holds(heading(X),T-1), occurs(turnLeft,T-1).
holds(heading((X+3)\4),T) :- holds(heading(X),T-1), occurs(turnRight,T-1).

% generate n actions - check this again. some constraints needed to be added
{occurs(A,T) : action(A), timestep(T)} = n.

%constraints: exactly one action per timestep
:- occurs(A,T), occurs(B,T), B!=A.
:- not occurs(_,T), timestep(T), T!=n.

%agent position needs to be defined for every timestep
:-  not holds(on(_,_),T) : timestep(T).

%end on different field
:- holds(on(A,B),n) : on(A,B).

%dont land on unsafe field
:- wumpusPossible(X,Y) : holds(on(X,Y),_).
:- trapPossible(X,Y) : holds(on(X,Y),_).
