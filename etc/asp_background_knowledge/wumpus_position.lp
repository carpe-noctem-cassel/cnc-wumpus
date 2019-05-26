#program wumpus_position.

#external on(X,Y) : field(X,Y).

%heading(up) :- heading(0).
%heading(left) :- heading(1). 
%heading(down) :- heading(2).
%heading(right) :- heading(3).

%define field ahead of agent
%fieldAhead(X-1,Y) :- field(X,Y), field(X-1,Y), heading(up), on(X,Y).
%fieldAhead(X+1,Y) :- field(X,Y), field(X+1,Y), heading(down), on(X,Y).
%fieldAhead(X,Y+1) :- field(X,Y), field(X,Y+1), heading(right), on(X,Y).
%fieldAhead(X,Y-1) :- field(X,Y), field(X,Y-1), heading(left), on(X,Y).

%define field left/right of agent 
%leftOfMe(X,Y+1) :- field(X,Y), field(X,Y+1), heading(down), on(X,Y).
%leftOfMe(X,Y-1) :- field(X,Y), field(X,Y-1), heading(up), on(X,Y).
%leftOfMe(X+1,Y) :- field(X,Y), field(X+1,Y), heading(left), on(X,Y).
%leftOfMe(X-1,Y) :- field(X,Y), field(X-1,Y), heading(right), on(X,Y).

%rightOfMe(X,Y+1) :- field(X,Y), field(X,Y+1), heading(up), on(X,Y).
%rightOfMe(X,Y-1) :- field(X,Y), field(X,Y-1), heading(down), on(X,Y).
%rightOfMe(X+1,Y) :- field(X,Y), field(X+1,Y), heading(right), on(X,Y).
%rightOfMe(X-1,Y) :- field(X,Y), field(X-1,Y), heading(left), on(X,Y).
