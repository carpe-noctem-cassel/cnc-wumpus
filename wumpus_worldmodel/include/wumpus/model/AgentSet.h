#pragma once

#include <set>
#include <model/Agent.h>



namespace eval {
    class AgentSet {
    public:
        auto cmp = [](std::shared_ptr<model::Agent> a, std::shared_ptr<model::Agent> b) { return a->id < b->id};
        std::set<int, decltype(cmp)> s(cmp);
        std::set<std::shared_ptr<model::Agent>, > agents;
    };
}