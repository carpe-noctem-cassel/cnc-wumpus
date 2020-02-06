#program wumpus_perception.

#external visited(X,Y) : field(X,Y).
#external glitter(X,Y) : field(X,Y).

#external goal(A,X,Y) : field(X,Y), agent(A).

#external haveGold(A) : agent(A).

%%wumpus and trap cannot be on the same field
%-trap(X,Y) :- field(X,Y), wumpus(X,Y).
%-wumpus(X,Y) :- field(X,Y), trap(X,Y).

%%danger 
%safe(X,Y) :- field(X,Y), not danger(X,Y).
%danger(X,Y) :- wumpus(X,Y).
%danger(X,Y) :- trap(X,Y).

%leftBlocked :- leftOfMe(X,Y), not safe(X,Y).
%rightBlocked :- rightOfMe(X,Y), not safe(X,Y).

