#program wumpus_externals.

%externals
#external on(X,Y) : field(X,Y).
#external goal(X,Y) : field(X,Y).
#external initial(X,Y) : field(X,Y).
#external visited(X,Y) : field(X,Y).
#external glitter(X,Y) : field(X,Y).
#external drafty(X,Y) : field(X,Y).
#external stinky(X,Y) : field(X,Y).
#external blacklist(X,Y) : field(X,Y).
#external shotAt(X,Y) : field(X,Y).
#external heading(0..3).
#external goalHeading(0..3).
#external wumpusBlocksMoves.
#external unsafeMovesAllowed.

%#show unsafeMovesAllowed/0.
%#show wumpusBlocksMoves/0.
%#show goalHeading/1.
%#show shotAt/2.
%#show goal/2.
%#show on/2.
%#show glitter/2.
