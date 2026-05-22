#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"
PYTHON_BIN="${PYTHON:-python3}"
DEFAULT_MIRRORS=(
  "https://pypi.tuna.tsinghua.edu.cn/simple"
  "https://mirrors.aliyun.com/pypi/simple/"
  "https://pypi.org/simple"
)
DEFAULT_HOSTS=(
  "pypi.tuna.tsinghua.edu.cn"
  "mirrors.aliyun.com"
  "pypi.org"
)

create_venv() {
  if [[ ! -d "${VENV_DIR}" ]]; then
    echo "Creating virtual environment at ${VENV_DIR}"
    "${PYTHON_BIN}" -m venv "${VENV_DIR}"
  else
    echo "Using existing virtual environment at ${VENV_DIR}"
  fi
}

activate_venv() {
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
}

install_requirements() {
  local mirrors=()
  local hosts=()
  local i

  if [[ -n "${PIP_INDEX_URL:-}" ]]; then
    mirrors=("${PIP_INDEX_URL}")
    hosts=("${PIP_TRUSTED_HOST:-$(printf '%s' "${PIP_INDEX_URL}" | sed -E 's#https?://([^/]+)/?.*#\1#')}")
  else
    mirrors=("${DEFAULT_MIRRORS[@]}")
    hosts=("${DEFAULT_HOSTS[@]}")
  fi

  export PIP_DISABLE_PIP_VERSION_CHECK=1

  for i in "${!mirrors[@]}"; do
    local mirror="${mirrors[$i]}"
    local host="${hosts[$i]}"

    echo "Trying pip mirror: ${mirror}"
    if python -m pip install --upgrade pip --index-url "${mirror}" --trusted-host "${host}" \
      && python -m pip install -r "${ROOT_DIR}/requirements.txt" --index-url "${mirror}" --trusted-host "${host}"; then
      echo "Dependencies installed from ${mirror}"
      return 0
    fi

    echo "Mirror failed: ${mirror}"
  done

  echo "All configured package mirrors failed." >&2
  return 1
}

main() {
  create_venv
  activate_venv
  install_requirements

  if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo
    echo "Environment is active in the current shell."
    echo "Python: $(command -v python)"
  else
    echo
    echo "Environment is ready."
    echo "Activate it with:"
    echo "source \"${VENV_DIR}/bin/activate\""
  fi
}

main "$@"
