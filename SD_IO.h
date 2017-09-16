
#include "FS.h"
#include "SD.h"
#include "SPI.h"
class SD_IO{
/*
 * Connect the SD card to the following pins:
 *
 * SD Card | SPI |  ESP32 IO |  ESP32 Pin
 *    D2     -      -           -
 *    D3     CS     IO 5        PIN 29
 *    CMD    MOSI   IO 23       PIN 37
 *    VDD    3.3V   3.3V        
 *    CLK    CLK    IO 18       PIN 30
 *    VSS    GND    GND         
 *    D0     MISO   IO 19       PIN 31
 *    D1     -      -           
 */
 public:
  void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
  void createDir(fs::FS &fs, const char * path);
  void removeDir(fs::FS &fs, const char * path);
  void readFile(fs::FS &fs, const char * path);
  void writeFile(fs::FS &fs, const char * path, const char * message);
  void appendFile(fs::FS &fs, const char * path, const char * message);
  void renameFile(fs::FS &fs, const char * path1, const char * path2);
  void deleteFile(fs::FS &fs, const char * path);
  void testFileIO(fs::FS &fs, const char * path);
  void Init();
  void Test();
  void write(const byte * message, int len);
  void openFile(fs::FS &fs, const char * path);
  void closeFile();
 private:
  File rec_file;
}; // class MPU9250