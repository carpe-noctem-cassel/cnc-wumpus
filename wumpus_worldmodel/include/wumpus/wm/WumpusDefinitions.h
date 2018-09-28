#pragma once
#include "Coordinates.h"

namespace wumpus
{
namespace wm
{

struct ScreamInfo
{
    Coordinates shotFrom;
};

struct GlobalInfo
{
    bool bumpHeard;
    bool exited;
    bool goldFound;
    bool isDead;
    bool notAllowed;
};

struct TurnInfo
{
    bool isMyTurn;
};
}
}
