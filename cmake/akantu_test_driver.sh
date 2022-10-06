#!/bin/bash

set -o errexit
set -o pipefail

show_help() {
    cat << EOF
Usage: ${0##*/} -n NAME -e EXECUTABLE [-p MPI_WRAPPER] [-s SCRIPT_FILE]
          [-r REFERENCE_FILE] [-w WORKING_DIR] [ARGS]
Execute the test in the good configuration according to the options given

    -e EXECUTABLE     Main executable of the test
    -n NAME           Name of the test
    -p MPI_WRAPPER    Executes the test for multiple parallel configuration
    -s SCRIPT_FILE    Script to execute after the execution of the test to
                      postprocess the results
    -v VALGRIND_PATH  Running the test using valgrind
    -r REFERENCE_FILE Reference file to compare with if the name of the file
                      contains a <nb_proc> this will be used for the different
                      configuration when -p is given
    -w WORKING_DIR    The directory in which to execute the test
    -E ENVIRONMENT_FILE File to source before running tests
    -h                Print this helps
EOF
}

full_redirect() {
  local nproc=$1
  shift
  local name=$1
  shift

  local sout=".lastout"
  local serr=".lasterr"
  if [ "${nproc}" -ne 0 ]; then
    sout="-${nproc}${sout}"
    serr="-${nproc}${serr}"
  fi
  echo "Run $*"
  ( ($* | tee "${name}${sout}") 3>&1 1>&2 2>&3 | tee "${name}${serr}") 3>&1 1>&2 2>&3

  lastout="${name}${sout}"
}

name=
executable=
parallel=
postprocess_script=
reference=
working_dir=
envi=
parallel_processes="2"
valgrind=""

while :
do
  case "$1" in
    -e)
      executable=$2
      shift 2
      ;;
    -E)
      envi="$2"
      shift 2
      ;;
    -h | --help)
      show_help
      exit 0
      ;;
    -n)
      name="$2"
      shift 2
      ;;
    -N)
      parallel_processes="$2"
      shift 2
      ;;
    -p)
      parallel="$2"
      shift 2
      ;;
    -r)
      reference="$2"
      shift 2
      ;;
    -s)
      postprocess_script="$2"
      shift 2
      ;;
    -w)
      working_dir="$2"
      shift 2
      ;;
    -v)
      valgrind="$2"
      shift 2
      ;;
    --) # End of all options
      shift
      break
      ;;
    -*)
      echo "Error: Unknown option: $1" >&2
      show_help
      exit 1
      ;;
    *) #No more options
      break
      ;;
  esac
done

_args=$*

if [ -n "${envi}" ]; then
  source "${envi}"
fi

if [ -z "${name}" ] || [ -z "${executable}" ]; then
  echo "Missing executable or name"
  show_help
  exit 1
fi

if [ -n "${working_dir}" ]; then
#    current_directory=$PWD
  echo "Entering directory ${working_dir}"
  cd "${working_dir}"
fi

if [ -z "${parallel}" ]; then
  echo "Executing the test ${name}"
  full_redirect 0 "${name}" "${valgrind} ${executable} ${_args}"
else
  #for i in ${parallel_processes}; do
  i=${parallel_processes}
  echo "Executing the test ${name} for ${i} procs"
  full_redirect "$i" "${name}"_"$i" "${parallel} ${i} ${valgrind} ${executable} ${_args}"
  #done
fi

if [ -n "${postprocess_script}" ]; then
  echo "Executing the test ${name} post-processing"
  full_redirect 0 "${name}_pp" "./${postprocess_script}"
fi

if [ -n "${reference}" ]; then
  echo "Comparing last generated output to the reference file"
  diff -w "${lastout}" "${reference}"
fi
