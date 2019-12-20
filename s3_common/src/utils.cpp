#include <string>

std::string GetFileName(const std::string& file_path)
{
    char sep = '/';
    size_t index = file_path.find_last_of(sep);
    if (index == std::string::npos) {
        return file_path;
    }
    return file_path.substr(index+1);
}

std::string GenerateObjectKey(const std::string& file_path, std::string prefix)
{
    if (!prefix.empty() && prefix.back() != '/') {
        prefix += "/";
    }
    return prefix + GetFileName(file_path);
}