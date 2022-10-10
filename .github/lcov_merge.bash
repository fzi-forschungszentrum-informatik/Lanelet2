#!/bin/bash
set -ex
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LANELET2_ROOT=$(dirname $SCRIPT_DIR)
WORKSPACE_ROOT=$(dirname $(dirname $LANELET2_ROOT))

src_dirs=""
for d in ${LANELET2_ROOT}/*/ ; do
    [ "$(basename $d)" = ".github" ] && continue
    [ "$(basename $d)" = "lanelet2" ] && continue
    src_dirs="${src_dirs} ${d}*"
done

lcov_filtered_files=""
for d in ${LANELET2_ROOT}/*/ ; do
    [ "$(basename $d)" = ".github" ] && continue
    [ "$(basename $d)" = "lanelet2" ] && continue
    pkg="$(basename $d)"
    build_dir="${WORKSPACE_ROOT}/build/${pkg}"
    
    file="${build_dir}/mrt_coverage/full_coverage.lcov"
    if [ -f "$file" ]; then
        filtered_path="${build_dir}/mrt_coverage/full_coverage_filtered.lcov"

        set -f
        lcov -o ${filtered_path} -e ${file}${src_dirs}
        set +f

        lcov_filtered_files="${lcov_filtered_files} -a ${filtered_path}"
    fi;
done

mkdir -p "${WORKSPACE_ROOT}/lcov"
lcov -o "${WORKSPACE_ROOT}/lcov/full_coverage.lcov" ${lcov_filtered_files}
