#program wumpus_goal.



distX(N) :- #sum{|X-A| : goal(X,Y), on(A,B)} = N.
distY(N) :- #sum{|Y-B| : goal(X,Y), on(A,B)} = N.
dist(N) :- #sum{X : distX(X); Y : distY(Y)} = N.

%the goal position should not be far away TODO should not consume a lot of actions
#minimize {N@1 : dist(N)}.

%if possible, the goal position should be considered safe
:~ safe(X,Y).
