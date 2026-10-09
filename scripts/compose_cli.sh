#!/usr/bin/env bash
# docker compose command shared by the headless smoke and flight scripts.
# COMPOSE_PROJECT_NAME is passed through, so two stacks can run at once
# and `down` only removes that project.

compose_setup() {
  local file profile old_ifs
  COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
  COMPOSE=(docker compose)
  if [ -n "${COMPOSE_PROJECT_NAME:-}" ]; then
    COMPOSE+=(-p "${COMPOSE_PROJECT_NAME}")
  fi
  old_ifs="${IFS}"
  IFS=':'
  local -a file_arr
  read -r -a file_arr <<< "${COMPOSE_FILES}"
  IFS="${old_ifs}"
  for file in "${file_arr[@]}"; do
    COMPOSE+=(-f "${ROOT}/${file}")
  done

  PROFILE_ARGS=()
  if [ -n "${COMPOSE_PROFILES:-}" ]; then
    IFS=','
    local -a profiles
    read -r -a profiles <<< "${COMPOSE_PROFILES}"
    IFS="${old_ifs}"
    for profile in "${profiles[@]}"; do
      profile="${profile// /}"
      if [ -n "${profile}" ]; then
        PROFILE_ARGS+=(--profile "${profile}")
      fi
    done
  fi

  # compose_stack.sh reads SERVICES after sourcing this file.
  # shellcheck disable=SC2034
  SERVICES=()
  if [ -n "${COMPOSE_SERVICES:-}" ]; then
    # shellcheck disable=SC2206,SC2034
    SERVICES=(${COMPOSE_SERVICES})
  fi
}

compose_ps_status() {
  "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" ps --format '{{.Service}} {{.Status}}' || true
}

compose_exec() {
  local service="$1"
  shift
  # Close stdin. An open stdin lets exec consume the caller's remaining input,
  # which drops every parameter command after the first.
  "${COMPOSE[@]}" exec -T "${service}" "$@" < /dev/null
}

# GNU timeout execs a program. compose_exec is a function, so timeout
# cannot call it. This expands the docker compose command, then times that out.
compose_exec_timeout() {
  local seconds="$1"
  local service="$2"
  shift 2
  timeout "${seconds}" "${COMPOSE[@]}" exec -T "${service}" "$@" < /dev/null
}
