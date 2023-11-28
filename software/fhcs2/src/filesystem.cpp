#include "filesystem.h"

namespace filesystem
{
  fs::FS &fs = LittleFS;

  void init(void)
  {
    if (!LittleFS.begin(true))
    {
      Serial.println("LittleFS Mount Failed");
      return;
    }
    listDir("/", 3);
  }

  void listDir(const char *dirname, uint8_t levels)
  {
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
      Serial.println("- failed to open directory");
      return;
    }
    if (!root.isDirectory())
    {
      Serial.println(" - not a directory");
      return;
    }

    File file = root.openNextFile();
    while (file)
    {
      if (file.isDirectory())
      {
        Serial.print("  DIR : ");

        Serial.print(file.name());
        time_t t = file.getLastWrite();
        struct tm *tmstruct = localtime(&t);
        Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);

        if (levels)
        {
          listDir(file.name(), levels - 1);
        }
      }
      else
      {
        Serial.print("  FILE: ");
        Serial.print(file.name());
        Serial.print("  SIZE: ");

        Serial.print(file.size());
        time_t t = file.getLastWrite();
        struct tm *tmstruct = localtime(&t);
        Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
      }
      file = root.openNextFile();
    }
  }

  void createDir(const char *path)
  {
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
      Serial.println("Dir created");
    }
    else
    {
      Serial.println("mkdir failed");
    }
  }

  void removeDir(const char *path)
  {
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path))
    {
      Serial.println("Dir removed");
    }
    else
    {
      Serial.println("rmdir failed");
    }
  }

  String readFile(const char *path)
  {
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
      Serial.println("- failed to open file for reading");
      return String();
    }

    String fileContent;
    if (file.available())
    {
      fileContent = file.readString();
    }
    return fileContent;
  }

  void writeFile(const char *path, const char *message)
  {
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
      Serial.println("- failed to open file for writing");
      return;
    }
    if (file.print(message))
    {
      Serial.println("- file written");
    }
    else
    {
      Serial.println("- write failed");
    }
    file.close();
  }

  void appendFile(const char *path, const char *message)
  {
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
      Serial.println("- failed to open file for appending");
      return;
    }
    if (file.print(message))
    {
      Serial.println("- message appended");
    }
    else
    {
      Serial.println("- append failed");
    }
    file.close();
  }

  void renameFile(const char *path1, const char *path2)
  {
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2))
    {
      Serial.println("- file renamed");
    }
    else
    {
      Serial.println("- rename failed");
    }
  }

  void deleteFile(const char *path)
  {
    Serial.printf("Deleting file: %s\r\n", path);
    if (fs.remove(path))
    {
      Serial.println("- file deleted");
    }
    else
    {
      Serial.println("- delete failed");
    }
  }
} /* namespace filesystem */