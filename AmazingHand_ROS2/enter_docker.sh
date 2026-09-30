#!/bin/bash
set -euo pipefail

container_name=amazing_hand_container

# 使用主機目前的桌面顯示位置，避免把可能變動的 DISPLAY 寫死。
if [[ -z "${DISPLAY:-}" ]]; then
    echo "主機未設定 DISPLAY。請從主機桌面的終端機執行此腳本。" >&2
    exit 1
fi

if ! docker container inspect "$container_name" >/dev/null; then
    echo "無法存取容器。若尚未建立，請先執行 bash run_docker.sh。" >&2
    exit 1
fi

xhost +si:localuser:root

if [[ "$(docker inspect -f '{{.State.Running}}' "$container_name")" != "true" ]]; then
    docker start "$container_name" >/dev/null
fi

exec docker exec -it \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    "$container_name" /bin/bash -c '
        # run_docker.sh 掛載整個專案，Compose 則只掛載 amazing_hand 子目錄。
        for api_root in /root/amazing_hand/amazing_hand/midas_hand_api /root/amazing_hand/midas_hand_api; do
            if [[ -f "$api_root/midas_hand_api/__init__.py" ]]; then
                export PYTHONPATH="$api_root${PYTHONPATH:+:$PYTHONPATH}"
                break
            fi
        done
        exec /bin/bash
    '
