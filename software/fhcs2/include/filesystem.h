#ifndef __FILE_SYSTEM_LITTLE_FS__
#define __FILE_SYSTEM_LITTLE_FS__

#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>

namespace filesystem
{
  extern void init(void);
  extern void listDir(const char *dirname, uint8_t levels);
  extern void createDir(const char *path);
  extern void removeDir(const char *path);
  extern String readFile(const char *path);
  extern void writeFile(const char *path, const char *message);
  extern void appendFile(const char *path, const char *message);
  extern void renameFile(const char *path1, const char *path2);
  extern void deleteFile(const char *path);
}

#endif /* __FILE_SYSTEM_LITTLE_FS__ */