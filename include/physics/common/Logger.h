#ifndef PHYSICS_LOGGER_H
#define PHYSICS_LOGGER_H

#include <sstream>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>

namespace physics {

class Logger {

  private:
    /* -- Attributes -- */

    /* Output destination file */
    std::string file;

    /* -- Methods -- */

    /* Retrieve the current timestamp */
    std::string getTime() const;

    /* Retrieve a formatted log message */
    std::string getFormattedMessage(const std::string& message) const;

  public:
    /* -- Methods -- */

    /* Constructor */
    Logger(const std::string& file = "");

    /* Destructor */
    ~Logger() = default;

    /* Log message to destination file */
    void log(const std::string& message);
};

inline Logger::Logger(const std::string& file) : file(file) {}

/* Retrieve the current timestamp */
inline std::string Logger::getTime() const {
  std::time_t now = std::time(nullptr);
  std::tm timeInfo;

  if(localtime_s(&timeInfo, &now) != 0) {
    return "ERR";
  }

  std::stringstream ss;
  ss << std::put_time(&timeInfo, "%Y-%m-%d %H:%M:%S");;
  return ss.str();
}

/* Retrieve a formatted log message */
inline std::string Logger::getFormattedMessage(const std::string& message) const {
  return "[" + getTime() + "]" + " " +  message;
}

/* Log message to destination file */
inline void Logger::log(const std::string& message) {
  std::string formattedMessage = getFormattedMessage(message);

  if(!file.empty()) {
    std::ofstream logStream(file, std::ios::app);
    logStream << formattedMessage << std::endl;
  }
  else {
    std::cout << formattedMessage << std::endl;
  }
}

}

#endif