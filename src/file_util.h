#ifndef _FILE_UTIL_H_
#define _FILE_UTIL_H_

#include <fstream>
#include <string>

inline bool IsFileExist(const std::string& file_path) {
  std::ifstream f(file_path.c_str());
  return f.good();
}

#endif  // _FILE_UTIL_H_