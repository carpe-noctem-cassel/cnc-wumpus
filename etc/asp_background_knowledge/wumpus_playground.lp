%%definitions
#program wumpus_playground(n).
fieldSize(n).
%%this encoding of the coordinates corresponds to the wumpus simulator
field(0..n-1, 0..n-1).
direction(0..3).

possibleObjective(explore;huntWumpus;shoot;goHome;leave;moveToGoldField;idle;collectGold;fetchOtherAgent).


%define field adjacency
fieldAdjacent(X,Y,X+1,Y) :- field(X,Y), field(X+1,Y).
fieldAdjacent(X,Y,X-1,Y) :- field(X,Y), field(X-1,Y).
fieldAdjacent(X,Y,X,Y+1) :- field(X,Y), field(X,Y+1).
fieldAdjacent(X,Y,X,Y-1) :- field(X,Y), field(X,Y-1).

%actions
action(move;turnLeft;turnRight;shoot;pickup;leave).

%for planning query
moveAction(move;turnLeft;turnRight).
