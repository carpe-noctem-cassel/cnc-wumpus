#program wumpus_stenchPerception. 

%%when a field is stinky, a wumpus can be at the neighbor fields
wumpus(X,Y) :- fieldAdjacent(A,B,X,Y), stinky(A,B), not withoutWumpus(X,Y).
%1 <= {wumpus(A-1,B), wumpus(A,B-1), wumpus(A+1,B), wumpus(A,B+1)} < 5 :-  fieldAdjacent(A,B,X,Y), stinky(A,B), not withoutWumpus(X,Y). 

%%if a field is known not to be stinky, there are no adjacent wumpi right now.
withoutWumpus(X,Y) :- fieldAdjacent(A,B,X,Y), explored(A,B), not stinky(A,B).

%%if i can visit a field, there can be no wumpus TODO only for static wumpi!
withoutWumpus(X,Y) :- explored(X,Y).

%field is without wumpus, when it was shot at
withoutWumpus(X,Y) :- shotAt(X,Y), me(A), not arrow(A).

%wumpus blocks safe moves 

%wumpusBlocksSafeMoves :- wumpus(X,Y),fieldAdjacent(X,Y,A,B), not explored(A,B).
%wumpusBlocksMoves :- wumpus(X,Y),fieldAdjacent(X,Y,A,B), not explored(A,B).
