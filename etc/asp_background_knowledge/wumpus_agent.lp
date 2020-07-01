%defines existance of an agent
#program wumpus_agent(id).
#external agent(id).
#external shot(A) : agent(A).

%externals having to do with agents
#external haveGold(id) : agent(A).
%#external goal(A,X,Y) : field(X,Y), agent(A).
#external me(id).
#external objective(A,O) : agent(A), possibleObjective(O).
#external position(A,X,Y) : agent(A), field(X,Y).
#external arrow(A) : agent(A).
#external exhausted(A) : agent(A).
#external hasSafePathToGold(A) : agent(A).
#external wumpusBlocksAgent(X,Y,A) : field(X,Y), agent(A).
#external trapBlocksAgent(X,Y,A) : field(X,Y), agent(A).