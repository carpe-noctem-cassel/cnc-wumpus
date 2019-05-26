#pragma once
namespace aspkb
{
/**
 * Strategies for integrating Information into the ASP knowledgebase
 * @example Agent receives an external update to their position in a grid world. The old position is no longer valid, so use FALSIFY_OLD_VALUES.
 * @example Agent learns that coffee can be drank from cups. This is unlikely to change, use INSERT_TRUE
 */
enum Strategy
{
    INSERT_TRUE,
    FALSIFY_OLD_VALUES
};
}