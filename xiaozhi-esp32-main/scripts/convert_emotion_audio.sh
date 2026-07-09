#!/bin/bash
# Convert emotion MP3 files to OGG for ESP32 firmware embedding
# Selects up to 5 files per emotion, prioritizing mid-size files

SRC="/home/wzh/audio_mao/emotion_sorted"
DST="/home/wzh/program/git/xiaoXiong_4G/xiaozhi-esp32-main/main/assets/common"

declare -A EMOTION_MAP=(
    ["快乐"]="happy"
    ["悲伤"]="sad"
    ["愤怒"]="angry"
    ["恐惧"]="fear"
    ["惊讶"]="surprise"
    ["厌恶"]="disgust"
    ["中性"]="neutral"
)

MAX_FILES=5

for cn_name in "${!EMOTION_MAP[@]}"; do
    en_name="${EMOTION_MAP[$cn_name]}"
    src_dir="$SRC/$cn_name"
    count=0

    echo "=== $cn_name → $en_name ==="

    # Sort by file size, pick middle-range files (skip smallest and largest)
    files=($(ls -lS "$src_dir"/*.mp3 2>/dev/null | awk '{print $5, $NF}' | sort -n | awk '{print $2}'))
    total=${#files[@]}

    if [ "$total" -eq 0 ]; then
        echo "  No files found, skipping"
        continue
    fi

    # Calculate start index to center the selection
    take=$((total < MAX_FILES ? total : MAX_FILES))
    start=$(( (total - take) / 2 ))

    for ((i=start; i<start+take && i<total; i++)); do
        src_file="${files[$i]}"
        count=$((count + 1))
        dst_file="$DST/emotion_${en_name}_${count}.ogg"

        echo "  Converting: $(basename "$src_file") → emotion_${en_name}_${count}.ogg"
        ffmpeg -y -i "$src_file" -c:a libvorbis -q:a 3 -ar 16000 -ac 1 "$dst_file" 2>/dev/null
        if [ $? -eq 0 ]; then
            size=$(stat -c%s "$dst_file")
            echo "    OK: ${size} bytes"
        else
            echo "    FAILED!"
        fi
    done
    echo "  Total: $count files"
    echo
done

echo "=== Summary ==="
ls -lh "$DST"/emotion_*.ogg 2>/dev/null | awk '{print $5, $NF}'
echo "Total files: $(ls "$DST"/emotion_*.ogg 2>/dev/null | wc -l)"
