#program wumpus_stenchPerception. 

%%when a field is stinky, a wumpus can be at the neighbor fields
wumpus(X,Y) :- fieldAdjacent(A,B,X,Y), stinky(A,B), not withoutWumpus(X,Y).

%%if a field is known not to be stinky, there are no adjacent wumpi right now.
withoutWumpus(X,Y) :- fieldAdjacent(A,B,X,Y), visited(A,B), not stinky(A,B).

%%if i can visit a field, there can be no wumpus TODO only for static wumpi!
withoutWumpus(X,Y) :- visited(X,Y).

%field is without wumpus, when it was shot at
withoutWumpus(X,Y) :- shotAt(X,Y).

%wumpus blocks safe moves 

%wumpusBlocksSafeMoves :- wumpus(X,Y),fieldAdjacent(X,Y,A,B), not visited(A,B).
%wumpusBlocksMoves :- wumpus(X,Y),fieldAdjacent(X,Y,A,B), not visited(A,B).
