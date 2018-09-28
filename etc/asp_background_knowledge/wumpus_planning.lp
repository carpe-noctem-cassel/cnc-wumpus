% horizon must be set. example: #const n = 6.
timestep(0..n-1).

%Rules for moving in direction of current heading (right, left, up, down)
holds(on(X,Y+1),T+1) :- occurs(move,T), holds(heading(right),T), field(X,Y), field(X,Y+1), holds(on(X,Y),T), T < n. 
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(right),T), field(X,Y), field(X,Y+1), holds(on(X,Y),T), T < n. 

holds(on(X,Y-1),T+1) :- occurs(move,T), holds(heading(left),T), field(X,Y), field(X,Y-1), holds(on(X,Y),T), T < n.
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(left),T), field(X,Y), field(X,Y-1), holds(on(X,Y),T), T < n.

holds(on(X+1,Y),T+1) :- occurs(move,T), holds(heading(up),T), field(X,Y), field(X+1,Y), holds(on(X,Y),T), T < n.
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(up),T), field(X,Y), field(X+1,Y), holds(on(X,Y),T), T < n.

holds(on(X-1,Y),T+1) :- occurs(move,T), holds(heading(down),T), field(X,Y), field(X-1,Y), holds(on(X,Y),T), T < n.
-holds(on(X,Y),T+1) :- occurs(move,T), holds(heading(down),T), field(X,Y), field(X-1,Y), holds(on(X,Y),T), T < n.


%Rules for Turning according to current heading
holds(heading(left),T+1) :- holds(heading(up),T), timestep(T), occurs(turnLeft,T), T < n.
-holds(heading(up),T+1) :- holds(heading(up),T), timestep(T), occurs(turnLeft,T), T < n.

holds(heading(up),T+1) :- holds(heading(right),T), timestep(T) ,occurs(turnLeft,T), T < n.
-holds(heading(right),T+1) :- holds(heading(right),T), timestep(T) ,occurs(turnLeft,T), T < n.

holds(heading(down),T+1) :- holds(heading(left),T), timestep(T) ,occurs(turnLeft,T), T < n.
-holds(heading(left),T+1) :- holds(heading(left),T), timestep(T) ,occurs(turnLeft,T), T < n.

holds(heading(right),T+1) :- holds(heading(down),T), timestep(T) ,occurs(turnLeft,T), T < n.
-holds(heading(down),T+1) :- holds(heading(down),T), timestep(T) ,occurs(turnLeft,T), T < n.



holds(heading(right),T+1) :- holds(heading(up),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(up),T+1) :- holds(heading(up),T), timestep(T), occurs(turnRight,T), T < n.

holds(heading(down),T+1) :- holds(heading(right),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(right),T+1) :- holds(heading(right),T), timestep(T), occurs(turnRight,T), T < n.

holds(heading(up),T+1) :- holds(heading(left),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(left),T+1) :- holds(heading(left),T), timestep(T), occurs(turnRight,T), T < n.

holds(heading(left),T+1) :- holds(heading(down),T), timestep(T), occurs(turnRight,T), T < n.
-holds(heading(down),T+1) :- holds(heading(down),T), timestep(T), occurs(turnRight,T), T < n.

%%inertia - things stay as they are unless changed 
holds(on(X,Y),T+1) :- holds(on(X,Y),T), not -holds(on(X,Y),T+1), T < n.
-holds(on(X,Y),T+1) :- -holds(on(X,Y),T), not holds(on(X,Y),T+1), T < n.
holds(heading(X),T+1) :- holds(heading(X),T), not -holds(heading(X),T+1), T < n.
-holds(heading(X),T+1) :- -holds(heading(X),T), not holds(heading(X),T+1), T < n.




%position/heading of agent on a field, can be changed by movement -> is "fluent"
fluent(defined, on(X,Y)).
fluent(defined, heading(X)).

%%occurence of actions general
%occurs(A,T) :- action(A), timestep(T), not -occurs(A,T).
%%CWA
%%-occurs(A,T) :- not occurs(A,T), action(A), timestep(T).
%%not more than 1 action at a given timestep
-occurs(B,T) :- occurs(A,T), timestep(T), action(A), action(B), A != B.

%% for all timesteps, each varying relation is defined
%holds(on(X,Y),T) :- timestep(T), not -holds(on(X,Y),T), field(X,Y).
%%CWA
%%-holds(F,T) :- not holds(F,T), fluent(F), timestep(T).
%-holds(on(X,Y),T) :- not holds(on(X,Y),T), field(X,Y), timestep(T).


%% only allow one state of being
%:- holds(on(A,B), T), holds(on(E,F), T), A != E, field(A,B), field(E,F).
%:- holds(on(A,B), T), holds(on(E,F), T), B != F, field(A,B), field(E,F).
%:- holds(heading(X),T), holds(heading(Y),T), X != Y, heading(X), heading(Y).



%%Planning: set goal(T) :- holds(desired, T).
% example: goal(T) :- holds(on(2,2),T).
success :- goal(T), T <=n.
:- not success.
1{occurs(A,T) : action(A)}1 :- not goal(T), timestep(T), T < n.
occurs(A,T) | -occurs(A,T) :- action(A) , timestep(T), T<n.
