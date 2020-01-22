#include <string>
#include <cstddef>

std::string GetFileName(const std::string& file_path)
{
  char sep = '/';
  size_t index = file_path.find_last_of(sep);
  if (index == std::string::npos) {
    return file_path;
  }
  return file_path.substr(index+1);
}

std::string GenerateObjectKey(const std::string& file_path, const std::string& prefix)
{
  std::string sep;
  if (!prefix.empty() && prefix.back() != '/') {
    sep = "/";
  }
  return prefix + sep + GetFileName(file_path);
}
