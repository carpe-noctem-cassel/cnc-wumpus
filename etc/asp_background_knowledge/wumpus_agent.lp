%defines existance of an agent
#program wumpus_agent(id).
agent(id).

%externals having to do with agents
#external haveGold(A) : agent(A).
%#external goal(A,X,Y) : field(X,Y), agent(A).
#external me(id).
#external objective(A,O) : agent(A), possibleObjective(O).
#external arrow(A) : agent(A).
#external exhausted(A) : agent(A).
