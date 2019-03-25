/*
6dof-stewduino
Copyright (C) 2018  Philippe Desrosiers

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Logger.h"

Logger::LogLevel Logger::level = TRACE;

void Logger::_log_va_list(const LogLevel level, const char* fmt, va_list args){
  static const char* lvlStrings[6] = {
    "TRACE",
    "DEBUG",
    "INFO",
    "WARN",
    "ERROR",
    "FATAL"
  };

  char buffer[256];
  // va_start (args, fmt);
  vsprintf (buffer,fmt, args);
  Serial.printf("[%s] - %s\n",lvlStrings[level],buffer);
  // va_end (args);
}

void Logger::log(const LogLevel level, const char* fmt, ...){
  if(level >= Logger::level){
    va_list args;
    va_start(args,fmt);
    _log_va_list(level,fmt,args);
    va_end(args);
  }
}

void Logger::trace(const char * fmt, ...){
  if(TRACE >= Logger::level){
    va_list args;
    va_start(args, fmt);
    _log_va_list(TRACE,fmt, args);
    va_end(args);
  }
}

void Logger::debug(const char * fmt, ...){
  if(DEBUG >= Logger::level){
    va_list args;
    va_start(args, fmt);
    _log_va_list(DEBUG,fmt, args);
    va_end(args);
  }
}

void Logger::info(const char * fmt, ...){
  if(INFO >= Logger::level){
    va_list args;
    va_start(args, fmt);
    _log_va_list(INFO,fmt, args);
    va_end(args);
  }
}

void Logger::warn(const char * fmt, ...){
  if(WARN >= Logger::level){
    va_list args;
    va_start(args, fmt);
    _log_va_list(WARN,fmt, args);
    va_end(args);
  }
}

void Logger::error(const char * fmt, ...){
  if(ERROR >= Logger::level){
    va_list args;
    va_start(args, fmt);
    _log_va_list(ERROR,fmt, args);
    va_end(args);
  }
}

void Logger::fatal(const char * fmt, ...){
  if(FATAL >= Logger::level){
    va_list args;
    va_start(args, fmt);
    _log_va_list(FATAL,fmt, args);
    va_end(args);
  }
}
