#include "wumpus/wm/Playground.h"
#include <iostream>
namespace wumpus
{
namespace wm
{

Playground::Playground(WumpusWorldModel* wm) : wm(wm)
{
	this->playgroundSize = -1;
}

Playground::~Playground()
{
    // TODO Auto-generated destructor stub
}

void Playground::addAgent(std::shared_ptr<wumpus::wm::WumpusAgent> agent) {
	std::lock_guard<std::mutex> lock(this->agentMtx);
	this->agents[agent->id] = agent;
}
std::shared_ptr<wumpus::wm::WumpusAgent> Playground::getAgentById(int id) {
	std::lock_guard<std::mutex> lock(this->agentMtx);
	auto it = this->agents.find(id);
	if(it != this->agents.end()) {
		return it->second;
	}
	return nullptr;
}

std::shared_ptr<wumpus::wm::Field> Playground::getField(wumpus::wm::Coordinates coords) {
	std::lock_guard<std::mutex> lock(this->fieldMtx);
	auto it = this->fields.find(std::make_pair(coords.x,coords.y));

	if(it != this->fields.end()) {
		return it->second;
	}
	return nullptr;
}

/**
 * Sets field size and creates fields
 */
void Playground::initializePlayground(int size) {
	std::lock_guard<std::mutex> lock(this->fieldMtx);
	this->playgroundSize = size;
	for(int i = 0; i < size; ++i) {
		for(int j = 0; j < size; ++j) {
			auto field = std::make_shared<wumpus::wm::Field>(i,j);
			this->fields[std::make_pair(i,j)] = field;
		}
	}
}

int Playground::getPlaygroundSize() {
	return this->playgroundSize;
}

int Playground::getNumberOfFields() {
	return this->playgroundSize * this->playgroundSize;
}


} /* namespace wm */
} /* namespace wumpus */
