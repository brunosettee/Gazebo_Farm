#!/bin/bash

BASE="/home/lucas-alves/Desktop/New_Dataset_LARS"
ROBO_INFO="robo_info.txt"
WORLD_TRANSLATION="world_translation.txt"

for i in $(seq -w 10 17); do
    MAPA="${BASE}/mapa_${i}"
    INFO="${MAPA}/info-file/"
    MESH="${MAPA}/mesh.dae"
    COORD="${MAPA}/coordinates.txt"
    TEXTURA="${MAPA}/textura_semantic.png"
    OUT="${MAPA}/semantic-segmentation/"

    # Cria o diretório de saída se não existir
    if [ ! -d "$OUT" ]; then
        mkdir -p "$OUT"
        echo "[CRIADO] $OUT"
    else
        echo "[OK] $OUT já existe"
    fi

    echo "[EXECUTANDO] python semantic-render.py $INFO camera_info.txt $MESH $COORD $TEXTURA $ROBO_INFO $WORLD_TRANSLATION $OUT"
    python semantic-render.py "$INFO" camera_info.txt "$MESH" "$COORD" "$TEXTURA" "$ROBO_INFO" "$WORLD_TRANSLATION" "$OUT"
done

