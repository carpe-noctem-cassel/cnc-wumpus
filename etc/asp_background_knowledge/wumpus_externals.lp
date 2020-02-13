#program wumpus_externals.

%externals
#external on(X,Y) : field(X,Y).
#external goal(X,Y) : field(X,Y).
#external initial(X,Y) : field(X,Y).
#external visited(X,Y) : field(X,Y).
#external explored(X,Y) : field(X,Y).
#external glitter(X,Y) : field(X,Y).
#external drafty(X,Y) : field(X,Y).
#external stinky(X,Y) : field(X,Y).
#external shotAt(X,Y) : field(X,Y).
#external heading(X) : direction(X).
#external goalHeading(X) : direction(X).
#external wumpusBlocksMoves.
#external unsafeMovesAllowed.
#external shot.

%#show unsafeMovesAllowed/0.
%#show wumpusBlocksMoves/0.
%#show goalHeading/1.
%#show shotAt/2.
%#show goal/2.
%#show on/2.
%#show glitter/2.


%#program glitterSection(n,m).
%#external glitter(n,m).

%#program stuff(n,m).
%#external is(n,m).

%für n = 123123 und m = Stift oder Haus oder Maus -> endlos

%q(is(n,m)):-q(stuffFromConceptNet).
