#program wumpus_draftPerception. 
%#external drafty(X,Y) : field(X,Y).

%%traps, analog to wumpus
%%when a field is drafty, a trap can be at the neighbor fields
trap(X,Y) :- fieldAdjacent(A,B,X,Y), drafty(A,B), not withoutTrap(X,Y).
%%if a field is known not to be drafty, there are no adjacent traps.
withoutTrap(X,Y) :- fieldAdjacent(A,B,X,Y), explored(A,B), not drafty(A,B).
withoutTrap(X,Y) :- explored(X,Y).
