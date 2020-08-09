#program wumpus_draftPerception. 
%#external drafty(X,Y) : field(X,Y).

%%traps, analog to wumpus
%%when a field is drafty, a trap can be at the neighbor fields
trapPossible(X,Y) :- fieldAdjacent(A,B,X,Y), drafty(A,B), not withoutTrap(X,Y).
trap(X,Y) :- trapPossible(X,Y), otherAgentDiedOn(X,Y).
%%if a field is known not to be drafty, there are no adjacent traps.
withoutTrap(X,Y) :- fieldAdjacent(A,B,X,Y), explored(A,B), not drafty(A,B).
withoutTrap(X,Y) :- explored(X,Y).

%generatesBreeze(X,Y,A,B) :- trapPossible(X,Y), drafty(A,B), fieldAdjacent(A,B,X,Y), field(X,Y), field(A,B).

%allAdjacentExplored(X,Y) :- field(X,Y), explored(A,B) : fieldAdjacent(X,Y,A,B), field(A,B).

%generatesBrezeCount(X,Y,A,B,N) :- field(X,Y), field(A,B), N = #count{1,X,Y,A,B : generatesBreeze(X,Y,A,B)}.

%trap(X,Y) :- generatesBreeze(X,Y,A,B), allAdjacentExplored(X,Y), generatesBreezeCount(X,Y,A,B,1), field(X,Y), field(A,B).


