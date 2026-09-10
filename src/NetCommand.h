#pragma once

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

namespace NetCommand {

// Команды управления изгибным стендом:
//   START - запустить программу циклов (из IDLE).
//   STOP  - прервать прогон и вернуться в центр 90° (из RUNNING).
//   HOME  - вернуться домой / восстановиться (из ERROR или IDLE - повторный homing;
//           из RUNNING эквивалентно STOP -> возврат домой).
enum class Command { NONE, START, STOP, HOME };

inline bool ieq(const char* a, const char* b) {
    while (*a && *b) {
        if (tolower(static_cast<unsigned char>(*a)) !=
            tolower(static_cast<unsigned char>(*b))) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

inline Command parseCommand(const char* s) {
    if (!s) return Command::NONE;
    if (ieq(s, "start")) return Command::START;
    if (ieq(s, "stop")) return Command::STOP;
    if (ieq(s, "home")) return Command::HOME;
    return Command::NONE;
}

inline const char* commandName(Command c) {
    switch (c) {
        case Command::START: return "START";
        case Command::STOP: return "STOP";
        case Command::HOME: return "HOME";
        default:            return "NONE";
    }
}

}  // namespace NetCommand
