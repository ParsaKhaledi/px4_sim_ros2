#!/usr/bin/env bash
# Load versions.env and .env without clobbering variables already set.
# registry, px4TAG, and the other names are assigned by sourcing those files.
# shellcheck disable=SC2154

load_repo_env() {
  local root="$1"
  local file line key
  for file in "${root}/versions.env" "${root}/.env"; do
    [ -f "${file}" ] || continue
    while IFS= read -r line || [ -n "${line}" ]; do
      line="${line%%#*}"
      line="${line#"${line%%[![:space:]]*}"}"
      line="${line%"${line##*[![:space:]]}"}"
      [ -z "${line}" ] && continue
      key="${line%%=*}"
      if [ -z "${!key+x}" ] || [ -z "${!key}" ]; then
        # shellcheck disable=SC2163
        export "${line?}"
      fi
    done < "${file}"
  done

  if [ -z "${PX4_IMAGE:-}" ]; then
    export PX4_IMAGE="${registry}/${IMAGE_NAME}:${px4TAG}"
  fi
  if [ -z "${PX4_GPU_IMAGE:-}" ]; then
    export PX4_GPU_IMAGE="${registryGPU:-${registry}}/${IMAGE_NAME}:${px4GPUTAG}"
  fi
}
