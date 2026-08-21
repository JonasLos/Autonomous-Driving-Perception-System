#!/bin/bash

source "$(dirname "${BASH_SOURCE[0]}")/lib/topics.sh"

# Path to topics.yaml -- derived from this script's location, overridable with TOPIC_FILE=.
TOPIC_FILE="$(default_topic_file)"

require_yq || exit 1

# === Extract all topic strings under "topics" (v4 yq syntax) ===
echo "Parsing topics from $TOPIC_FILE..."
TOPICS=$(yq eval '.topics | .. | select(tag == "!!str")' "$TOPIC_FILE" | sort -u)

if [[ -z "$TOPICS" ]]; then
    echo "[ERROR] No topics found in the YAML file."
    exit 1
fi

# === Check Frequencies ===
echo "Gathering topic frequencies..."
echo "------------------------------"

while IFS= read -r topic; do
  echo "Topic: $topic"
  timeout 3s ros2 topic hz "$topic" 2>/dev/null | grep -i "average rate" || echo "  No data or not publishing"
  echo "------------------------------"
done <<< "$TOPICS"


