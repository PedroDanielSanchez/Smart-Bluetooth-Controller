// Log level. Ensure log_level_name() is updated for newly added levels
typedef enum LogLevel
{
    LOG_LEVEL_ALL = 1, // Log all messages
    LOG_LEVEL_TRACE = 1,
    LOG_LEVEL_INFO = 30,
    LOG_LEVEL_WARN = 40,
    LOG_LEVEL_ERROR = 50,
    LOG_LEVEL_PANIC = 60,
    LOG_LEVEL_NONE = 70, // Do not log any messages
    // Compatibility levels
    DEFAULT_LEVEL = 0,
    ALL_LEVEL = LOG_LEVEL_ALL,
    TRACE_LEVEL = LOG_LEVEL_TRACE,
    LOG_LEVEL = LOG_LEVEL_TRACE,   // Deprecated
    DEBUG_LEVEL = LOG_LEVEL_TRACE, // Deprecated
    INFO_LEVEL = LOG_LEVEL_INFO,
    WARN_LEVEL = LOG_LEVEL_WARN,
    ERROR_LEVEL = LOG_LEVEL_ERROR,
    PANIC_LEVEL = LOG_LEVEL_PANIC,
    NO_LOG_LEVEL = LOG_LEVEL_NONE
} LogLevel;