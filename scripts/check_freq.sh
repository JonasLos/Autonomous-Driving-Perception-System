#!/bin/bash

# Path to topics.yaml
TOPIC_FILE="/home/dev/Documents/Autonomous-Driving-Perception-System/src/topics.yaml"

# === Check if yq is installed ===
if ! command -v yq &> /dev/null; then
    echo "[ERROR] 'yq' is not installed. Install it using: sudo snap install yq"
    exit 1
fi

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


