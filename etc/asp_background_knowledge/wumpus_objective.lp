#program wumpus_objective.

objective(explore) :- not glitter(_,_), not haveGold(A), me(A), not wumpusBlocksMoves.

objective(huntWumpus) :- wumpusBlocksMoves, arrow(A), me(A).

objective(shoot) :- on(X,Y), goal(X,Y), heading(H), goalHeading(H), arrow(A), me(A).

objective(goHome) :- haveGold(A), me(A), not on(X,Y),  initial(X,Y).

objective(leave) :- haveGold(A), me(A), on(X,Y), initial(X,Y).

objective(moveToGoldField) :- glitter(X,Y), not on(X,Y), not haveGold(A), me(A).

objective(collectGold) :- glitter(X,Y), on(X,Y), not haveGold(A), me(A).

