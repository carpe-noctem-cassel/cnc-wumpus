#program wumpus_perception.

#external visited(X,Y) : field(X,Y).
#external explored(X,Y) : field(X,Y).
#external glitter(X,Y) : field(X,Y).

#external goal(A,X,Y) : field(X,Y), agent(A).

#external haveGold(A) : agent(A).

%%wumpus and trap cannot be on the same field
%-trapPossible(X,Y) :- field(X,Y), wumpusPossible(X,Y).
%-wumpusPossible(X,Y) :- field(X,Y), trapPossible(X,Y).

%%danger 
%safe(X,Y) :- field(X,Y), not danger(X,Y).
%danger(X,Y) :- wumpusPossible(X,Y).
%danger(X,Y) :- trapPossible(X,Y).

%leftBlocked :- leftOfMe(X,Y), not safe(X,Y).
%rightBlocked :- rightOfMe(X,Y), not safe(X,Y).

