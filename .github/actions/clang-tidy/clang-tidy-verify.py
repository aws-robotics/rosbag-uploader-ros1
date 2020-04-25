#!/usr/bin/env python3

import os
import sys
import re
# import time
import argparse
import multiprocessing
import multiprocessing.pool
import subprocess
import json
from datetime import datetime


def read_args():
  parser = argparse.ArgumentParser(description="a script that calls clang-tidy-6.0 on each file"
                                   "in each compilation database found in subdirectories")
  parser.add_argument("-j", type=int, default=multiprocessing.cpu_count(),
                      help="number of clang-tidy instances to run in parallel")
  parser.add_argument("--outputdir", default="./clang-tidy-output/",
                      help="path to output artifacts")
  args = parser.parse_args()
  return args


def init_log_paths():
  user_input = read_args()

  args = dict()
  args["num_workers"]       = user_input.j
  args["output_dir"]        = user_input.outputdir + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
  args["logfile"]           = os.path.join(args["output_dir"], "clang-tidy-verify.log")
  args["clang_stdout"]      = os.path.join(args["output_dir"], "clang-stdout")
  args["clang_stderr"]      = os.path.join(args["output_dir"], "clang-stderr")
  args["clang_tidy_bin"]    = "clang-tidy-6.0"
  args["clang_tidy_config"] = """
---
Checks: >
  bugprone-*,
  cert-*,
  clang-diagnostic-*,
  clang-analyzer-*,
  cppcoreguidelines-*,
  google-*,
  hicpp-*,
  misc-*,
  modernize-*,
  performance-*,
  readability-*,
  -cert-err58-cpp,
  -cppcoreguidelines-pro-bounds-array-to-pointer-decay,
  -cppcoreguidelines-pro-bounds-constant-array-index,
  -cppcoreguidelines-pro-type-vararg,
  -google-runtime-references,
  -hicpp-no-array-decay,
  -hicpp-special-member-functions,
  -hicpp-vararg,
  -misc-misplaced-const,
  -modernize-pass-by-value,
  -readability-avoid-const-params-in-decls,
  -readability-else-after-return

WarningsAsErrors: "*"

CheckOptions:
  - { key: cppcoreguidelines-special-member-functions.AllowSoleDefaultDtor,      value: 1 }
  - { key: cppcoreguidelines-special-member-functions.AllowMissingMoveFunctions, value: 1 }
  - { key: readability-identifier-naming.NamespaceCase,               value: CamelCase }
  - { key: readability-identifier-naming.ClassCase,                   value: CamelCase }
  - { key: readability-identifier-naming.StructCase,                  value: CamelCase }
  - { key: readability-identifier-naming.TemplateParameterCase,       value: CamelCase }
  - { key: readability-identifier-naming.FunctionCase,                value: CamelCase }
  - { key: readability-identifier-naming.EnumCase,                    value: CamelCase }
  - { key: readability-identifier-naming.EnumConstantCase,            value: UPPER_CASE }
  - { key: readability-identifier-naming.ConstexprVariableCase,       value: CamelCase }
  - { key: readability-identifier-naming.ConstexprVariablePrefix,     value: k }
  - { key: readability-identifier-naming.GlobalConstantCase,          value: CamelCase }
  - { key: readability-identifier-naming.GlobalConstantPrefix,        value: k }
  - { key: readability-identifier-naming.GlobalConstantPointerCase,   value: CamelCase }
  - { key: readability-identifier-naming.GlobalConstantPointerPrefix, value: k }
  - { key: readability-identifier-naming.StaticConstantCase,          value: CamelCase }
  - { key: readability-identifier-naming.StaticConstantPrefix,        value: k }
  - { key: readability-identifier-naming.LocalConstantCase,           value: aNy_CasE }
  - { key: readability-identifier-naming.VariableCase,                value: lower_case }
  - { key: readability-identifier-naming.GlobalVariablePrefix,        value: g_ }
  - { key: readability-identifier-naming.PrivateMemberSuffix,         value: _ }
  - { key: readability-identifier-naming.ProtectedMemberSuffix,       value: _ }
"""

  return args


def get_compilation_db_paths(curr_dir, visited_dirs, log):
  visited_dirs.add(curr_dir)

  paths = list()
  for item in os.listdir(curr_dir):
    item_path = os.path.realpath(os.path.join(curr_dir, item))
    if(os.path.isfile(item_path)):
      if(item == "compile_commands.json"):
        paths.append(item_path)
        log.write(item_path + "\n")
    elif(os.path.isdir(item_path)):
      if not(item_path in visited_dirs):
        paths += get_compilation_db_paths(item_path, visited_dirs, log)

  return paths


def update_clang_tidy_config(clang_tidy_config):
  if (os.path.exists(".clang-tidy")):
    backup_filename = "clang-tidy.backup-" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    os.rename(".clang-tidy", backup_filename)
  clang_tidy_file = open(".clang-tidy", "w")
  clang_tidy_file.write(clang_tidy_config)
  clang_tidy_file.close()


def is_gtest_source(file_name):
  if(file_name == "gtest_main.cc" or file_name == "gtest-all.cc"
    or file_name == "gmock_main.cc" or file_name == "gmock-all.cc"):
    return True
  return False

def is_unittest_source(package_name, file_path):
  return ("%s/test/" % package_name) in file_path

def invoke_clang_tidy(clang_tidy_bin, package):
  package_dir = os.path.dirname(package)
  package_name = os.path.basename(package_dir)

  # args = ["-dump-config"]
  args = ["-header-filter", "include/%s/.*" % package_name, "-p", package_dir]
  stdout_content = bytearray()
  stderr_content = bytearray()

  db = json.load(open(package))
  for item in db:
    if is_gtest_source(os.path.basename(item["file"])):
      continue
    if is_unittest_source(package_name, item["file"]):
      continue

    clang_tidy_args = [clang_tidy_bin] + args + [item["file"]]

    output = subprocess.Popen(clang_tidy_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (stdout, stderr) = output.communicate()
    stdout_content.extend(stdout)
    stderr_content.extend(stderr)

  return (package_name, args, stdout_content, stderr_content)


def main():
  args = init_log_paths()

  pool = multiprocessing.pool.ThreadPool(args["num_workers"])
  os.makedirs(args["output_dir"], exist_ok=True)

  log = open(args["logfile"], "w")

  print("working directory: " + os.getcwd())
  log.write("working directory:\n")
  log.write(os.getcwd() + "\n")
  log.write(args["clang_tidy_config"] + "\n")
  update_clang_tidy_config(args["clang_tidy_config"])

  log.write("compilation database found at:\n")
  packages = get_compilation_db_paths(os.getcwd(), set(), log)
  results = list()
  for package in packages:
    print("linting " + os.path.dirname(package) + "...")
    results.append(pool.apply_async(invoke_clang_tidy, (args["clang_tidy_bin"], package)))
  pool.close()
  pool.join()
  print("linting finished\n\n")

  total_num_of_errors = 0
  outfile = open(args["clang_stdout"], "wb")
  errfile = open(args["clang_stderr"], "wb")
  for (package, result) in zip(packages, results):
    log.write("===============\n")
    log.write(package + "\n")
  
    (package_name, args, stdout_content, stderr_content) = result.get()
    outfile.write(stdout_content)
    errfile.write(stderr_content)

    print(stdout_content.decode())

    num_of_errors = len(re.findall("error:", stdout_content.decode("utf-8"), re.MULTILINE))
    print(package_name + " num_of_errors: " + str(num_of_errors) + "\n")
    log.write("num_of_errors: " + str(num_of_errors) + "\n")
    if num_of_errors > 0:
      log.write("\nfix (some of the) errors by running:\n")
      log.write("run-clang-tidy-6.0.py %s -fix\n\n" % " ".join(args))

    total_num_of_errors += num_of_errors
    # time.sleep(5)
  log.write("total num of errors: " + str(total_num_of_errors) + "\n")

  log.close()
  outfile.close()
  errfile.close()

  return total_num_of_errors


if __name__ == "__main__":
  num_errs = main()
  if num_errs == 0:
    sys.exit(0)
  else:
    sys.exit(1)
