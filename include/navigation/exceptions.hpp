/**
 * Custom exceptions for navigation system.
 */
#ifndef NAVIGATION_EXCEPTIONS_HPP
#define NAVIGATION_EXCEPTIONS_HPP

#include <exception>
#include <string>
#include <optional>

namespace navigation {

/**
 * Base exception for navigation system errors.
 */
class NavigationError : public std::exception {
protected:
    std::string message_;

public:
    explicit NavigationError(const std::string& message) : message_(message) {}

    virtual const char* what() const noexcept override {
        return message_.c_str();
    }
};

/**
 * Raised when buoy detection fails.
 */
class BuoyDetectionError : public NavigationError {
private:
    std::optional<std::string> buoy_type_;

public:
    explicit BuoyDetectionError(const std::string& message,
                               const std::optional<std::string>& buoy_type = std::nullopt)
        : NavigationError(message), buoy_type_(buoy_type) {}

    const std::optional<std::string>& getBuoyType() const { return buoy_type_; }
};

/**
 * Raised when waypoint planning fails.
 */
class WaypointPlanningError : public NavigationError {
private:
    std::optional<std::string> stage_;

public:
    explicit WaypointPlanningError(const std::string& message,
                                  const std::optional<std::string>& stage = std::nullopt)
        : NavigationError(message), stage_(stage) {}

    const std::optional<std::string>& getStage() const { return stage_; }
};

/**
 * Raised when MAVLink communication fails.
 */
class MAVLinkError : public NavigationError {
private:
    std::optional<std::string> command_;

public:
    explicit MAVLinkError(const std::string& message,
                         const std::optional<std::string>& command = std::nullopt)
        : NavigationError(message), command_(command) {}

    const std::optional<std::string>& getCommand() const { return command_; }
};

/**
 * Raised when configuration is invalid.
 */
class ConfigurationError : public NavigationError {
private:
    std::optional<std::string> parameter_;

public:
    explicit ConfigurationError(const std::string& message,
                               const std::optional<std::string>& parameter = std::nullopt)
        : NavigationError(message), parameter_(parameter) {}

    const std::optional<std::string>& getParameter() const { return parameter_; }
};

} // namespace navigation

#endif // NAVIGATION_EXCEPTIONS_HPP
