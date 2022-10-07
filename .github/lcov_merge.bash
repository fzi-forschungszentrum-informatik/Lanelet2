#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LANELET2_ROOT=$(dirname $SCRIPT_DIR)

src_dirs=""
for d in ${LANELET2_ROOT}/build/*/ ; do
    [ "$(basename $d)" = "catkin_tools_prebuild" ] && continue
    pkg="$(basename $d)"
    src_dir=$(find ${PWD}/src -type d -name ${pkg} -prune)
    src_dirs="${src_dirs} ${src_dir}/*"
done

lcov_filtered_files=""
for d in ${LANELET2_ROOT}/build/*/ ; do
    [ "$(basename $d)" = "catkin_tools_prebuild" ] && continue
    file="${d}mrt_coverage/full_coverage.lcov"
    if [ -f "$file" ]; then
        pkg="$(basename $d)"
        filtered_path="${d}mrt_coverage/full_coverage_filtered.lcov"

        set -f
        lcov -o ${filtered_path} -e ${file}${src_dirs}
        set +f

        lcov_filtered_files="${lcov_filtered_files} -a ${filtered_path}"
    fi;
done

mkdir /tmp/lcov
lcov -o /tmp/lcov/full_coverage.lcov ${lcov_filtered_files}