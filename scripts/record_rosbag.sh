#!/bin/bash

# === Path to topics.yaml ===
TOPIC_FILE="/home/dev/Documents/Autonomous-Driving-Perception-System/src/topics.yaml"

# === Check if yq is installed ===
if ! command -v yq &> /dev/null; then
    echo "[ERROR] 'yq' is not installed. Install it using: sudo apt install yq"
    exit 1
fi

# === Parse and Validate Input Categories ===
if [[ $# -eq 0 ]]; then
    echo "[INFO] No category arguments provided. Defaulting to 'raw'."
    CATEGORIES=("raw")
else
    CATEGORIES=("$@")
fi

ALL_TOPICS=""
INVALID_CATEGORIES=()

for CATEGORY in "${CATEGORIES[@]}"; do
    # Extract topic paths for this category
    TOPIC_PATHS=$(yq e ".categories.${CATEGORY}[]" "$TOPIC_FILE" 2>/dev/null)

    if [[ -z "$TOPIC_PATHS" ]]; then
        INVALID_CATEGORIES+=("$CATEGORY")
        continue
    fi

    while IFS= read -r path; do
        topic=$(yq e ".$path" "$TOPIC_FILE")
        ALL_TOPICS+=" $topic"
    done <<< "$TOPIC_PATHS"
done

# === Handle Invalid Categories ===
if [[ ${#INVALID_CATEGORIES[@]} -gt 0 ]]; then
    echo "[ERROR] Invalid categories: ${INVALID_CATEGORIES[*]}"
    echo "Valid categories are: $(yq e '.categories | keys | join(" ")' "$TOPIC_FILE")"
    exit 1
fi

# === Prompt for Metadata ===
read -p "Enter location [Unknown location]: " LOCATION
LOCATION=${LOCATION:-"Unknown location"}

read -p "Enter vehicle name/ID [Unknown vehicle]: " VEHICLE
VEHICLE=${VEHICLE:-"Unknown vehicle"}

read -p "Enter comments [No comments]: " COMMENTS
COMMENTS=${COMMENTS:-"No comments"}

read -p "Enter maneuver [No maneuver specified]: " MANEUVER
MANEUVER=${MANEUVER:-"No maneuver specified"}

read -p "Enter number of passengers [0]: " PASSENGERS
PASSENGERS=${PASSENGERS:-0}

read -p "Enter road type [Unknown]: " ROAD_TYPE
ROAD_TYPE=${ROAD_TYPE:-"Unknown"}

read -p "Enter road condition [Unknown]: " ROAD_CONDITION
ROAD_CONDITION=${ROAD_CONDITION:-"Unknown"}

# === Metadata String ===
JOINED_CATEGORIES=$(IFS=_ ; echo "${CATEGORIES[*]}")
METADATA="location: $LOCATION, vehicle: $VEHICLE, passengers: $PASSENGERS, road_type: $ROAD_TYPE, road_condition: $ROAD_CONDITION, comments: $COMMENTS, maneuver: $MANEUVER, categories: $JOINED_CATEGORIES"

# === File and Folder Setup ===
TIMESTAMP=$(date +'%Y-%m-%d_%H-%M-%S')
BAG_BASENAME="rosbag_${JOINED_CATEGORIES// /_}_${TIMESTAMP}"
BAG_NAME="$BAG_BASENAME.bag"
TXT_NAME="$BAG_BASENAME.txt"
SAVE_DIR="/media/dev/T9/${JOINED_CATEGORIES// /_}_$TIMESTAMP"
mkdir -p "$SAVE_DIR"

# === Write Metadata ===
{
    echo "Rosbag Name: $BAG_NAME"
    echo "Categories: $JOINED_CATEGORIES"
    echo "Location: $LOCATION"
    echo "Vehicle: $VEHICLE"
    echo "Number of Passengers: $PASSENGERS"
    echo "Road Type: $ROAD_TYPE"
    echo "Road Condition: $ROAD_CONDITION"
    echo "Comments: $COMMENTS"
    echo "Maneuver: $MANEUVER"
    echo "Recorded Topics:"
    echo "$ALL_TOPICS"
    echo "/rosbag_metadata"
} > "$SAVE_DIR/$TXT_NAME"

# === Publish Metadata to ROS ===
echo "[INFO] Publishing metadata to /rosbag_metadata"
rostopic pub -1 /rosbag_metadata std_msgs/String "data: \"$METADATA\"" &

sleep 1  # Give time for the message to be published

# === Record Rosbag ===
echo "[INFO] Starting rosbag recording..."
echo "Saving to: $SAVE_DIR/$BAG_NAME"
rosbag record -e --split --size=5000 -o "$SAVE_DIR/$BAG_BASENAME" $ALL_TOPICS /rosbag_metadata
