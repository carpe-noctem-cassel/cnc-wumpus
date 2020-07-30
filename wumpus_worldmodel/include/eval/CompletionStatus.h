#pragma once
#include <ostream>
namespace eval
{

enum CompletionStatus
{
    UNDEFINED,
    REJECTED,
    SUCCESS,
    FAILURE,
    TIMEOUT
};

inline std::ostream& operator<<(std::ostream& str, CompletionStatus completionStatus)
{
    switch (completionStatus) {
    case UNDEFINED:
        return str << "UNDEFINED";
    case REJECTED:
        return str << "REJECTED";
    case SUCCESS:
        return str << "EXITED";
    case FAILURE:
        return str << "DIED";
    case TIMEOUT:
        return str << "TIMEOUT";
    default:
        return str << (int) completionStatus;
    }
}
}
