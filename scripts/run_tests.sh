#!/usr/bin/env bash
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;034m'
DEFAULT='\e[39m'

failed_tests=0
test_dirs=$(find . -name tests)
# shellcheck disable=SC2086
echo -e "${BLUE}" found "${test_dirs//[$'\t\r\n']}" to format ${DEFAULT}
c=$(echo "$test_dirs" | wc -l)

for i in $(seq "$c"); do
  dir=$(echo "$test_dirs" | head -"$i" | tail -1)
  test_files="$(echo "$dir"/test_*.py)"
  for file_name in $test_files; do
      file_path=$(echo "$file_name" | sed 's/\.\///g')
      echo -e "${BLUE} testing $file_path ${DEFAULT}"
      dir_failed=$(pipenv run python -m unittest -v -f "$file_path" 2>&1 | grep -c "FAILED")
      failed_tests=$((  $failed_tests + $dir_failed ))
    if [ "$dir_failed" -gt "0" ]; then
      echo -e "${RED}" "$dir_failed" failed in this directory "${DEFAULT}"
    fi
  done
done
if [ "$failed_tests" -eq "0" ]; then
  echo -e "${GREEN} all tests passed!"
else
  echo -e "${RED} $failed_tests tests failed!"
  exit 1
fi
