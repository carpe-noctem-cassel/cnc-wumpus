#include "wumpus/wm/planning/SafePathExistsPlanner.h"
#include <wumpus/model/Agent.h>
#include <aspkb/Integrator.h>
#include <wumpus/WumpusWorldModel.h>
namespace wumpus
{
namespace wm
{
namespace planning
{

SafePathExistsPlanner::SafePathExistsPlanner(aspkb::Extractor* extractor,aspkb::Integrator* integrator)
        : Planner(extractor), integrator(integrator)
{
    auto sc = essentials::SystemConfig::getInstance();
    this->maxHorizonFactor = (*sc)[KB_CONFIG_NAME]->get<int>("maxHorizonFactor", NULL);
}
struct StartEndRep
{ // TODO implement cleaner
    const std::string fromXRep = "x";
    const std::string fromYRep = "y";
    const std::string toXRep = "a";
    const std::string toYRep = "b";
    const std::string undef = "UNDEFINED";

    std::map<std::string, std::string> reps = {
            std::make_pair(fromXRep, undef), std::make_pair(fromYRep, undef), std::make_pair(toXRep, undef), std::make_pair(toYRep, undef)};
};

void SafePathExistsPlanner::checkSafePathsExistsForOtherAgents(std::pair<int, int> from, const std::map<int, std::pair<int, int>>& tos)
{
    // assume correct placement TODO replace with struct or similar?
    auto startHorizon = 1;
    auto maxHorizon = this->maxHorizonFactor * this->wm->playground->getPlaygroundSize();
    std::vector<std::string> result;

    for (const auto& to : tos) {

        // trivial case - incremental approach doesn't work for this so far
        if (from.first == to.second.first && from.second == to.second.first) {
            result.push_back("start(" + std::to_string(from.first) + "," + std::to_string(from.second) + (")"));
            result.push_back("end(" + std::to_string(to.second.first) + "," + std::to_string(to.second.second) + (")"));
        } else {

            auto startEndRep = StartEndRep();
            startEndRep.reps.at(startEndRep.fromXRep) = std::to_string(from.first);
            startEndRep.reps.at(startEndRep.fromYRep) = std::to_string(from.second);
            startEndRep.reps.at(startEndRep.toXRep) = std::to_string(to.second.first);
            startEndRep.reps.at(startEndRep.toYRep) = std::to_string(to.second.second);

            auto externalPrefix =
                    "safePath" + std::to_string(from.first) + std::to_string(from.second) + std::to_string(to.second.first) + std::to_string(to.second.second);

            // create necessary problems if necessary
            if (this->safePathExistsForOtherAgentProblems.find(from) != this->safePathExistsForOtherAgentProblems.end()) {
                auto problemForStart = this->safePathExistsForOtherAgentProblems.at(from);

                if (problemForStart.find(to.second) == problemForStart.end()) {
                    std::cout << "Found from " << from.first << ", " << from.second << " but couldn't find to! " << to.second.first << ", " << to.second.second
                              << std::endl;
                    auto problem = std::make_shared<aspkb::IncrementalProblem>(aspkb::TermManager::getInstance().getSolver(),
                            std::vector<std::string>({"pathComplete(wildcard)"}), startEndRep.reps, KB_CONFIG_NAME, "safePathExistsIncrementalProblem",
                            externalPrefix, startHorizon, maxHorizon, false);

                    this->safePathExistsForOtherAgentProblems.at(from).emplace(to.second, problem);
                } else {
                    std::cout << "Found from  " << from.first << ", " << from.second << " and to!" << to.second.first << ", " << to.second.second << std::endl;
                }
            } else {
                std::cout << "Couldn't find from! " << from.first << ", " << from.second << "for to " << to.second.first << ", " << to.second.second
                          << std::endl;

                auto map = std::map<std::pair<int, int>, std::shared_ptr<aspkb::IncrementalProblem>>();
                auto problem = std::make_shared<aspkb::IncrementalProblem>(aspkb::TermManager::getInstance().getSolver(),
                        std::vector<std::string>({"pathComplete(wildcard)"}), startEndRep.reps, KB_CONFIG_NAME, "safePathExistsIncrementalProblem",
                        externalPrefix, startHorizon, maxHorizon, false);
                map.emplace(to.second, problem);
                this->safePathExistsForOtherAgentProblems.emplace(from, map);
            };

            std::string safePathQueryValue = "path(wildcard, wildcard, wildcard)";
            std::cout << "solving for from " << from.first << ", " << from.second << ", to:  " << to.second.first << ", " << to.second.second << std::endl;
            for (const auto& elem : this->safePathExistsForOtherAgentProblems) {
                std::cout << "problems map from: " << elem.first.first << ", " << elem.first.second << std::endl;
                for (const auto& innerElem : elem.second) {
                    std::cout << "problems map to: " << innerElem.first.first << ", " << innerElem.first.second << std::endl;
                }
            }

            // set externals for start
            std::stringstream ss;
            ss << "start(" << from.first << ", " << from.second << ")";
            auto start = ss.str();
            this->integrator->integrateInformationAsExternal(start, "safePathStart", true, aspkb::Strategy::INSERT_TRUE);
            ss.str("");
            ss << "end(" << to.second.first << ", " << to.second.second << ")";
            auto end = ss.str();
            this->integrator->integrateInformationAsExternal(end, "safePathEnd", true, aspkb::Strategy::INSERT_TRUE);
            this->integrator->applyChanges();

            // try to get result
            result = this->extractor->solveWithIncrementalExtensionQuery(this->safePathExistsForOtherAgentProblems.at(from).at(to.second));

            // reset externals
            this->integrator->integrateInformationAsExternal(start, "safePathStart", false, aspkb::Strategy::INSERT_TRUE);
            this->integrator->integrateInformationAsExternal(end, "safePathEnd", false, aspkb::Strategy::INSERT_TRUE);
            this->integrator->applyChanges();

            auto path = std::vector<std::pair<std::string, std::string>>();
        }

        if (!result.empty()) {
            std::cout << "PlanningModule: SAFEPATHEXISTS: result not empty " << std::endl;
            std::stringstream ss2;
            ss2 << "safePathExists(" << from.first << "," << from.second << "," << to.second.first << "," << to.second.second << ")";
            auto finalRep = ss2.str();
            std::cout << "SAFEPATHEXISTS: " << finalRep << std::endl;
            this->wm->playground->getAgentById(to.first)->updateHaveSafePathToGold();
            this->integrator->integrateInformationAsExternal(finalRep, "safePathExists", true, aspkb::Strategy::INSERT_TRUE);
            this->integrator->applyChanges();
        } else {
            //            std::cout << "PlanningModule: Should have a safe path for the first world" << std::endl;
            //            throw std::exception();
        }
    }
    // FIXME TODO implement handling result / setting of external etc
    //    throw std::exception();
}

} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */