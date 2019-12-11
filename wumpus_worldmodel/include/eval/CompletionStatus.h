namespace eval
{

enum CompletionStatus
{
    UNDEFINED,
    EXITED,
    DIED,
    TIMEOUT
};

inline std::ostream& operator<<(std::ostream& str, CompletionStatus completionStatus)
{
    switch (completionStatus) {
    case UNDEFINED:
        return str << "UNDEFINED";
    case EXITED:
        return str << "EXITED";
    case DIED:
        return str << "DIED";
    case TIMEOUT:
        return str << "TIMEOUT";
    default:
        return str << (int) completionStatus;
    }
}
}
