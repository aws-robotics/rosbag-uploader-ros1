/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#pragma once

#include <fstream>
#include <string>
#include <cstddef>

inline bool FileExists(const std::string& name)
{
    std::ifstream ifile(name);
    return ifile.good();
}

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