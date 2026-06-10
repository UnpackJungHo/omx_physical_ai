#!/usr/bin/env bash
# 모든 패키지의 build/<pkg>/compile_commands.json 을 워크스페이스 루트로 병합한다.
#
# clangd 는 --compile-commands-dir=${workspaceFolder} 로 루트의 단일
# compile_commands.json 만 본다. colcon 은 패키지별로 따로 생성할 뿐
# 루트 병합본을 만들어 주지 않으므로, 새 패키지를 빌드한 뒤 clangd 가
# 해당 .cpp/.hpp 의 컴파일 플래그를 알게 하려면 이 스크립트로 병합해야 한다.
#
# 사용법:
#   colcon build --symlink-install ...   # 먼저 빌드 (per-pkg compile_commands 생성)
#   ./tools/update_compile_commands.sh    # 루트로 병합
#
# 같은 file 이 여러 패키지에 나타나면 마지막(가장 최근 빌드) 항목을 남긴다.

set -euo pipefail

# 스크립트 위치 기준으로 워크스페이스 루트 결정 (어디서 실행해도 동작).
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${WS_ROOT}/build"
OUT="${WS_ROOT}/compile_commands.json"

if [ ! -d "${BUILD_DIR}" ]; then
  echo "error: build/ 디렉토리가 없습니다. 먼저 colcon build 를 실행하세요." >&2
  exit 1
fi

python3 - "${BUILD_DIR}" "${OUT}" <<'PY'
import glob
import json
import os
import sys

build_dir, out_path = sys.argv[1], sys.argv[2]

# file 경로 -> 컴파일 항목. dict 로 dedupe (나중 항목이 앞을 덮어씀).
merged = {}
sources = sorted(glob.glob(os.path.join(build_dir, "*", "compile_commands.json")))
for path in sources:
    with open(path) as f:
        try:
            entries = json.load(f)
        except json.JSONDecodeError as exc:
            print(f"warn: {path} 파싱 실패 ({exc}), 건너뜀", file=sys.stderr)
            continue
    for e in entries:
        key = e.get("file", "")
        if key:
            merged[key] = e

with open(out_path, "w") as f:
    json.dump(list(merged.values()), f, indent=2)

pkgs = sorted(os.path.basename(os.path.dirname(p)) for p in sources)
print(f"merged {len(merged)} entries from {len(sources)} package(s): {', '.join(pkgs)}")
print(f"-> {out_path}")
PY
