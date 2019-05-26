#include <WumpusSimulatorHeadless.h>

#include <WumpusEnums.h>
#include <model/Agent.h>
#include <model/GroundTile.h>
#include <model/Model.h>
#include <model/Movable.h>
#include <model/Wumpus.h>

#include <ros/master.h>

#include <QJsonDocument>
#include <QJsonObject>
#include <QtCore/QFile>
#include <memory>


namespace wumpus_simulator
{
namespace headless
{

WumpusSimulatorHeadless::WumpusSimulatorHeadless()
{
    this->model = nullptr;
    turnIndex = 0;

    spawnAgentSub = n.subscribe("/wumpus_simulator/SpawnAgentRequest", 10, &WumpusSimulatorHeadless::onSpawnAgent, (WumpusSimulatorHeadless*) this);
    actionSub = n.subscribe("/wumpus_simulator/ActionRequest", 10, &WumpusSimulatorHeadless::onAction, (WumpusSimulatorHeadless*) this);

    spawnAgentPub = n.advertise<wumpus_simulator::InitialPoseResponse>("/wumpus_simulator/SpawnAgentResponse", 10);
    actionPub = n.advertise<wumpus_simulator::ActionResponse>("/wumpus_simulator/ActionResponse", 10);

    spinner = new ros::AsyncSpinner(4);
    spinner->start();
    this->ready = false;
}

    WumpusSimulatorHeadless::~WumpusSimulatorHeadless()
{
}

/*void WumpusSimulatorHeadless::initPlugin(qt_gui_cpp::PluginContext& context)
{

        this->widget_ = new QWidget();
        this->widget_->setAttribute(Qt::WA_AlwaysShowToolTips, true);
        this->mainwindow.setupUi(this->widget_);

        if (context.serialNumber() > 1)
        {
                this->widget_->setWindowTitle(
                                this->widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(this->widget_);

        //Allow to show a inspector in the web view, nice inline debugging of javascript
        this->mainwindow.webView->page()->settings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        QWebInspector inspector;
        inspector.setPage(this->mainwindow.webView->page());
        inspector.setVisible(true);

        //Initialize web view
        this->connect(this->mainwindow.webView->page()->mainFrame(), SIGNAL(javaScriptWindowObjectCleared()), this,
                                        SLOT(addSimToJS()));
        this->mainwindow.webView->load(QUrl("qrc:///www/index.html"));
        this->connect(this, SIGNAL(modelChanged()), this, SLOT(callUpdatePlayground()));
        this->ready = false;
} */

Model* WumpusSimulatorHeadless::getModel()
{
    return this->model;
}

void WumpusSimulatorHeadless::createWorld(bool arrow, int wumpus, int traps, int size)
{

    std::cout << "WumpusSimulator: Creating world with: arrow: " << (arrow ? "true" : "false") << " wumpus count: " << wumpus << " trap count: " << traps
              << " field size: " << size << std::endl;
    // Init the playground
    this->model = Model::get();
    this->model->init(arrow, wumpus, traps, size);
    ready = true;
}

void WumpusSimulatorHeadless::loadWorld(std::string world)
{
    this->model = Model::get();
    this->turns.clear();
    // Open load file dialog to select a pregenerated wumpus world
    QString filename = QString::fromStdString(world);

    // Check if the user selected a correct file
    if (!filename.isNull()) {
        // Open file
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly)) {
            qWarning("Couldn't open save file.");
            return;
        }

        QByteArray saveData = file.readAll();
        QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));
        this->model->fromJSON(loadDoc.object());
        ready = true;
    }
}

void WumpusSimulatorHeadless::onSpawnAgent(InitialPoseRequestPtr msg)
{
    if (!ready) {
        return;
    }
    if (msg->agentId > 0) {
        placeAgent(msg->agentId, this->model->getAgentHasArrow());
    } else if (msg->agentId < 0) {
        possessWumpus(msg->agentId);
    } else {
        std::cout << "WumpusSimulator: ID = 0 not supported!" << std::endl;
    }
}

void WumpusSimulatorHeadless::onAction(ActionRequestPtr msg)
{
    if (!ready) {
        return;
    }
    if (turns.size() == 0) {
        return;
    }
    if (msg->agentId != this->turns.at(this->turnIndex)) {
        std::cout << "WumpusSimulator: agent is not allowed to move! It's agent's " << this->turns.at(this->turnIndex) << " turn!" << std::endl;
        return;
    }
    bool found = false;
    for (auto mov : this->model->movables) {
        if (msg->agentId == mov->getId()) {
            found = true;
            break;
        }
    }
    if (found) {
        if (msg->agentId > 0) {
            handleAction(msg);
        } else {
            handleWumpusAction(msg);
        }
    }
}

void WumpusSimulatorHeadless::possessWumpus(int wumpusId)
{
    for (int i = 0; i < model->movables.size(); i++) {
        if (this->model->movables.at(i)->getId() == wumpusId) {
            std::cout << "WumpusSimulator: Wumpus with this id already possessed!" << std::endl;
            return;
        }
    }
    bool available = false;
    for (auto mov : this->model->movables) {
        if (mov->getId() == 0) {
            mov->setId(wumpusId);
            turns.push_back(wumpusId);
            available = true;
            break;
        }
    }
    if (!available) {
        std::cout << "WumpusSimulator: no Wumpus available!" << std::endl;
    }
}

void WumpusSimulatorHeadless::placeAgent(int agentId, bool hasArrow)
{
    for (int i = 0; i < model->movables.size(); i++) {
        if (this->model->movables.at(i)->getId() == agentId) {
            std::cout << "WumpusSimulator: Agent with id " << agentId << " already placed!" << std::endl;
            return;
        }
    }
    /* initialize random seed: */
    srand(time(NULL));
    bool placed = false;
    InitialPoseResponse msg;
    int attempts = 0;
    while (!placed) {
        if (attempts > (pow(model->getPlayGroundSize(), 3))) {
            std::cout << "Abort! Cannot find empty tile to place agent on." << std::endl;
            break;
        }
        int randx = rand() % (this->model->getPlayGroundSize());
        int randy = rand() % (this->model->getPlayGroundSize());
        // random heading
        int randz = rand() % 4;

        auto tile = this->model->getPlayGround().at(randx).at(randy);
        if (!tile->getTrap() && !tile->hasMovable() && !tile->getGold() && !tile->getBreeze() && !tile->getStench() && !tile->getStartpoint()) {
            auto agent = std::make_shared<Agent>(this->model->getPlayGround().at(randx).at(randy));
            agent->setId(agentId);
            agent->setArrow(hasArrow);
            // agent->setHeading(WumpusEnums::heading::up);
            agent->setHeading(static_cast<WumpusEnums::heading>(randz));
            this->model->getPlayGround().at(randx).at(randy)->setMovable(agent);
            this->model->movables.push_back(agent);
            tile->setStartAgentID(agentId);
            tile->setStartpoint(true);
            placed = true;
            msg.x = randx;
            msg.y = randy;
            msg.agentId = agentId;
            msg.fieldSize = this->model->getPlayGroundSize();
            msg.hasArrow = hasArrow;
            msg.heading = agent->getHeading();
            this->spawnAgentPub.publish(msg);
            turns.push_back(agent->getId());
            if (turns.size() == 1) {
                ActionResponse msg2;
                msg2.x = agent->getTile()->getX();
                msg2.y = agent->getTile()->getY();
                msg2.agentId = agent->getId();
                msg2.heading = agent->getHeading();
                msg2.responses.push_back(WumpusEnums::responses::yourTurn);
                handlePerception(msg2, agent->getTile());
                this->actionPub.publish(msg2);
            }
        }
        attempts++;
    }
}

void WumpusSimulatorHeadless::handleTurnRight(ActionRequestPtr msg)
{
    ActionResponse response;
    auto agent = this->model->getAgentByID(msg->agentId);
    auto tmp = (((agent->getHeading() - 1) + 4) % 4);
    agent->setHeading((WumpusEnums::heading)(tmp));
    response.agentId = agent->getId();
    response.x = agent->getTile()->getX();
    response.y = agent->getTile()->getY();
    response.heading = tmp;
    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handleTurnLeft(ActionRequestPtr msg)
{
    ActionResponse response;
    auto agent = this->model->getAgentByID(msg->agentId);
    auto tmp = ((agent->getHeading() + 1) % 4);
    agent->setHeading((WumpusEnums::heading)(tmp));
    response.agentId = agent->getId();
    response.x = agent->getTile()->getX();
    response.y = agent->getTile()->getY();
    response.heading = tmp;
    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handleShoot(ActionRequestPtr msg)
{
    ActionResponse response;
    auto agent = this->model->getAgentByID(msg->agentId);
    response.agentId = agent->getId();
    response.x = agent->getTile()->getX();
    response.y = agent->getTile()->getY();
    response.heading = agent->getHeading();
    if (agent->hasArrow()) {
        if (agent->getHeading() == WumpusEnums::heading::left) {
            handleShootLeft(response, agent);
        } else if (agent->getHeading() == WumpusEnums::heading::right) {
            handleShootRight(response, agent);
        } else if (agent->getHeading() == WumpusEnums::heading::up) {
            handleShootUp(response, agent);
        } else {
            handleShootDown(response, agent);
        }
        agent->setArrow(false);
        handlePerception(response, agent->getTile());
    } else {
        response.responses.push_back(WumpusEnums::responses::notAllowed);
        handlePerception(response, agent->getTile());
    }
    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handlePickUpGold(ActionRequestPtr msg)
{
    ActionResponse response;
    auto agent = this->model->getAgentByID(msg->agentId);
    response.agentId = agent->getId();
    response.x = agent->getTile()->getX();
    response.y = agent->getTile()->getY();
    response.heading = agent->getHeading();
    if (agent->getTile()->getGold()) {
        response.responses.push_back(WumpusEnums::responses::goldFound);
        agent->setHasGold(true);
    } else {
        response.responses.push_back(WumpusEnums::responses::notAllowed);
    }
    handlePerception(response, agent->getTile());
    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handleExit(ActionRequestPtr msg)
{
    ActionResponse response;
    auto agent = this->model->getAgentByID(msg->agentId);
    response.agentId = agent->getId();
    response.x = agent->getTile()->getX();
    response.y = agent->getTile()->getY();
    response.heading = agent->getHeading();
    if (agent->getHasGold() && agent->getTile()->getStartAgentID() == agent->getId()) {
        response.responses.push_back(WumpusEnums::responses::exited);
        this->turns.erase(std::find(this->turns.begin(), this->turns.end(), agent->getId()));
        this->model->exit(agent);
    } else {
        response.responses.push_back(WumpusEnums::responses::notAllowed);
    }
    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handleMove(ActionRequestPtr msg)
{
    ActionResponse response;
    auto agent = this->model->getAgentByID(msg->agentId);
    response.agentId = agent->getId();
    int x = agent->getTile()->getX();
    int y = agent->getTile()->getY();
    if ((x == 0 && agent->getHeading() == WumpusEnums::heading::up) ||
            (x == this->model->getPlayGroundSize() - 1 && agent->getHeading() == WumpusEnums::heading::down) ||
            (y == 0 && agent->getHeading() == WumpusEnums::heading::left) ||
            (y == this->model->getPlayGroundSize() - 1 && agent->getHeading() == WumpusEnums::heading::right)) {
        response.x = x;
        response.y = y;
        response.heading = agent->getHeading();
        response.responses.push_back(WumpusEnums::responses::bump);
    } else {
        if (agent->getHeading() == WumpusEnums::heading::up) {
            x -= 1;
        } else if (agent->getHeading() == WumpusEnums::heading::down) {
            x += 1;
        } else if (agent->getHeading() == WumpusEnums::heading::left) {
            y -= 1;
        } else {
            y += 1;
        }

        response.x = x;
        response.y = y;
        response.heading = agent->getHeading();

        if (this->model->getTile(x, y)->getTrap() || this->model->getTile(x, y)->hasWumpus()) {
            this->killAgent(agent);
        } else if (this->model->getTile(x, y)->hasMovable() && !this->model->getTile(x, y)->hasWumpus()) {
            response.x = agent->getTile()->getX();
            response.y = agent->getTile()->getY();
            response.responses.push_back(WumpusEnums::responses::otherAgent);
        } else {
            this->model->removeAgent(agent);
            agent->setTile(this->model->getTile(x, y));
            agent->getTile()->setMovable(agent);
        }
    }
    auto tmp = this->model->getTile(x, y);
    handlePerception(response, tmp);

    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handleWumpusAction(ActionRequestPtr msg)
{
    if (!(msg->action == WumpusEnums::heading::up || msg->action == WumpusEnums::heading::down || msg->action == WumpusEnums::heading::left ||
                msg->action == WumpusEnums::heading::right)) {

        std::cout << "WumpusSimulator: unknown Action received" << std::endl;
        return;
    }

    ActionResponse response;
    auto wumpus = this->model->getWumpusByID(msg->agentId);
    response.agentId = wumpus->getId();
    int x = wumpus->getTile()->getX();
    int y = wumpus->getTile()->getY();
    if ((x == 0 && msg->action == WumpusEnums::heading::up) || (x == this->model->getPlayGroundSize() - 1 && msg->action == WumpusEnums::heading::down) ||
            (y == 0 && msg->action == WumpusEnums::heading::left) ||
            (y == this->model->getPlayGroundSize() - 1 && msg->action == WumpusEnums::heading::right)) {
        response.x = x;
        response.y = y;
        response.heading = WumpusEnums::heading::down;
        response.responses.push_back(WumpusEnums::responses::bump);
    } else {
        if (msg->action == WumpusEnums::heading::up) {
            x -= 1;
        } else if (msg->action == WumpusEnums::heading::down) {
            x += 1;
        } else if (msg->action == WumpusEnums::heading::left) {
            y -= 1;
        } else {
            y += 1;
        }

        response.x = x;
        response.y = y;
        response.heading = WumpusEnums::heading::down;

        if (this->model->getTile(x, y)->hasWumpus()) {
            response.x = wumpus->getTile()->getX();
            response.y = wumpus->getTile()->getY();
            response.responses.push_back(WumpusEnums::responses::otherAgent);
        }

        if (this->model->getTile(x, y)->hasMovable() && !this->model->getTile(x, y)->hasWumpus()) {
            auto tmp = std::dynamic_pointer_cast<Agent>(this->model->getTile(x, y)->getMovable());
            this->killAgent(tmp);
            response.responses.push_back(WumpusEnums::responses::killedAgent);
        }

        this->model->removeWumpus(wumpus);
        wumpus->setTile(this->model->getTile(x, y));
        wumpus->getTile()->setMovable(wumpus);
        for (auto mov : this->model->movables) {
            if (mov->getId() <= 0) {
                this->model->setStench(mov->getTile()->getX(), mov->getTile()->getY());
            }
        }
    }
    this->actionPub.publish(response);
    handleNextTurn();
}

void WumpusSimulatorHeadless::handleAction(ActionRequestPtr msg)
{

    switch (msg->action) {
    case WumpusEnums::actions::move: {
        handleMove(msg);
        break;
    }
    case WumpusEnums::actions::leave: {
        handleExit(msg);
        break;
    }
    case WumpusEnums::actions::pickUpGold: {
        handlePickUpGold(msg);
        break;
    }
    case WumpusEnums::actions::shoot: {
        handleShoot(msg);
        break;
    }
    case WumpusEnums::actions::turnLeft: {
        handleTurnLeft(msg);
        break;
    }
    case WumpusEnums::actions::turnRight: {
        handleTurnRight(msg);
        break;
    }

    default:
        std::cout << "WumpusSimulator: unknown Action received" << std::endl;
        break;
    }
    handleNextTurn();
}

void WumpusSimulatorHeadless::handleNextTurn()
{
    if (this->turns.size() == 0) {
        return;
    }
    getNext();
    ActionResponse response;
    auto id = this->turns.at(turnIndex);
    response.agentId = id;
    if (id > 0) {
        auto agent = this->model->getAgentByID(id);
        response.heading = agent->getHeading();
        auto tmp = agent->getTile();
        handlePerception(response, tmp);
        response.x = tmp->getX();
        response.y = tmp->getY();
    } else {
        auto wumpus = this->model->getWumpusByID(id);
        auto tmp = wumpus->getTile();
        response.x = tmp->getX();
        response.y = tmp->getY();
    }
    response.responses.push_back(WumpusEnums::responses::yourTurn);
    this->actionPub.publish(response);
}

void WumpusSimulatorHeadless::handlePerception(ActionResponse& msg, std::shared_ptr<GroundTile> tile)
{
    if (tile->getGold()) {
        msg.responses.push_back(WumpusEnums::responses::shiny);
    }
    if (tile->getBreeze()) {
        msg.responses.push_back(WumpusEnums::responses::drafty);
    }
    if (tile->getStench()) {
        msg.responses.push_back(WumpusEnums::responses::stinky);
    }
}

void WumpusSimulatorHeadless::handleShootLeft(ActionResponse& msg, std::shared_ptr<Agent> agent)
{
    bool wumpusDead = false;
    if (agent->getTile()->getY() == 0) {
        msg.responses.push_back(WumpusEnums::responses::silence);
    } else {
        for (int i = agent->getTile()->getY() - 1; i >= 0; i--) {
            if (this->getModel()->getPlayGround().at(agent->getTile()->getX()).at(i)->hasWumpus()) {
                wumpusDead = true;
                auto tmp = std::dynamic_pointer_cast<Wumpus>(this->getModel()->getPlayGround().at(agent->getTile()->getX()).at(i)->getMovable());
                killWumpus(tmp);
            }
        }
        if (wumpusDead) {
            msg.responses.push_back(WumpusEnums::responses::scream);
        } else {
            msg.responses.push_back(WumpusEnums::responses::silence);
        }
    }
}

void WumpusSimulatorHeadless::handleShootRight(ActionResponse& msg, std::shared_ptr<Agent> agent)
{
    bool wumpusDead = false;
    if (agent->getTile()->getY() == this->model->getPlayGroundSize() - 1) {
        msg.responses.push_back(WumpusEnums::responses::silence);
    } else {
        for (int i = agent->getTile()->getY() + 1; i <= this->model->getPlayGroundSize() - 1; i++) {
            if (this->getModel()->getPlayGround().at(agent->getTile()->getX()).at(i)->hasWumpus()) {
                wumpusDead = true;
                auto tmp = std::dynamic_pointer_cast<Wumpus>(this->getModel()->getPlayGround().at(agent->getTile()->getX()).at(i)->getMovable());
                killWumpus(tmp);
            }
        }
        if (wumpusDead) {
            msg.responses.push_back(WumpusEnums::responses::scream);
        } else {
            msg.responses.push_back(WumpusEnums::responses::silence);
        }
    }
}

void WumpusSimulatorHeadless::handleShootUp(ActionResponse& msg, std::shared_ptr<Agent> agent)
{
    bool wumpusDead = false;
    if (agent->getTile()->getX() == 0) {
        msg.responses.push_back(WumpusEnums::responses::silence);
    } else {
        for (int i = agent->getTile()->getX() - 1; i >= 0; i--) {
            if (this->getModel()->getPlayGround().at(i).at(agent->getTile()->getY())->hasWumpus()) {
                wumpusDead = true;
                auto tmp = std::dynamic_pointer_cast<Wumpus>(this->getModel()->getPlayGround().at(i).at(agent->getTile()->getY())->getMovable());
                killWumpus(tmp);
            }
        }
        if (wumpusDead) {
            msg.responses.push_back(WumpusEnums::responses::scream);
        } else {
            msg.responses.push_back(WumpusEnums::responses::silence);
        }
    }
}

void WumpusSimulatorHeadless::handleShootDown(ActionResponse& msg, std::shared_ptr<Agent> agent)
{
    bool wumpusDead = false;
    if (agent->getTile()->getX() == this->model->getPlayGroundSize() - 1) {
        msg.responses.push_back(WumpusEnums::responses::silence);
    } else {
        for (int i = agent->getTile()->getX() + 1; i <= this->model->getPlayGroundSize() - 1; i++) {
            if (this->getModel()->getPlayGround().at(i).at(agent->getTile()->getY())->hasWumpus()) {
                wumpusDead = true;
                auto tmp = std::dynamic_pointer_cast<Wumpus>(this->getModel()->getPlayGround().at(i).at(agent->getTile()->getY())->getMovable());
                killWumpus(tmp);
            }
        }
        if (wumpusDead) {
            msg.responses.push_back(WumpusEnums::responses::scream);
        } else {
            msg.responses.push_back(WumpusEnums::responses::silence);
        }
    }
}

void WumpusSimulatorHeadless::killWumpus(std::shared_ptr<Wumpus> wumpus)
{
    if (wumpus->getId() != 0) {
        ActionResponse response2;
        response2.agentId = wumpus->getId();
        response2.responses.push_back(WumpusEnums::responses::dead);
        this->actionPub.publish(response2);
        this->turns.erase(std::find(this->turns.begin(), this->turns.end(), wumpus->getId()));
    }
    this->getModel()->removeWumpus(wumpus);
    this->model->movables.erase(remove(this->model->movables.begin(), this->model->movables.end(), wumpus), this->model->movables.end());
    for (auto mov : this->model->movables) {
        if (mov->getId() <= 0) {
            this->model->setStench(mov->getTile()->getX(), mov->getTile()->getY());
        }
    }
}

void WumpusSimulatorHeadless::killAgent(std::shared_ptr<Agent> agent)
{
    this->turns.erase(std::find(this->turns.begin(), this->turns.end(), agent->getId()));
    ActionResponse response2;
    response2.agentId = agent->getId();
    response2.responses.push_back(WumpusEnums::responses::dead);
    this->actionPub.publish(response2);
    this->model->exit(agent);
}

void WumpusSimulatorHeadless::getNext()
{
    this->turnIndex++;
    if (this->turnIndex > this->turns.size()) {
        this->turnIndex = this->turns.size();
    }
    this->turnIndex = turnIndex % this->turns.size();
}
}
}
