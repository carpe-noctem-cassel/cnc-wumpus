exhaustedCount(N) :- N = #count{1, A : exhausted(A), agent(A)}.
agentCount(M) :- M = #count{1, X : agent(X)}.
allAgentsExhausted :- exhaustedCount(N), agentCount(M), N == M.