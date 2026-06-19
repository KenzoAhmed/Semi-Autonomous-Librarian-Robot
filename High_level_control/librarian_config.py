# librarian_config.py
#
# High-level configuration for the semi-autonomous librarian robot.
#
# IMPORTANT:
# The dictionary keys MUST exactly match the YOLO class names.
#
# Your YOLO model classes are:
#   1. how-to-write-a-love-story
#   2. lost-time-is-never-found-again
#   3. night-of-terror
#
# Each ESP path command already includes:
#   - going to the selected book shelf
#   - performing the required movement sequence
#   - returning back to home
#
# Therefore, the high-level controller only chooses:
#   path1 / path2 / path3
#
# Later Pi-to-ESP integration:
#   MOVE:path1  -> send "path1\n" to movement ESP
#   MOVE:path2  -> send "path2\n" to movement ESP
#   MOVE:path3  -> send "path3\n" to movement ESP

CONF_THRESHOLD = 0.70
REQUIRED_STABLE_DETECTIONS = 3


BOOKS = {
    "how-to-write-a-love-story": {
        "display_name": "How To Write A Love Story",
        "esp_path_command": "path1",
    },

    "lost-time-is-never-found-again": {
        "display_name": "Lost Time Is Never Found Again",
        "esp_path_command": "path2",
    },

    "night-of-terror": {
        "display_name": "Night Of Terror",
        "esp_path_command": "path3",
    },
}