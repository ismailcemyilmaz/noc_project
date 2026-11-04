#!/bin/sh
# Record the noc-tilthex ee_joint force_torque sensor with host receive timestamps.
#
#   usage: noc/experiments/log_ft.sh <out_file> [topic]      (default topic /noc/ee_ft)
#
# Output: one line per gz.msgs.Wrench message,
#   <host receive time, unix s> TAB <message JSON from gz topic --json-output>
# Lines starting with '#' are comments (start marker). Stop with Ctrl-C / SIGTERM.
# gz output is line-buffered (stdbuf) so the host time is taken when the message
# arrives, not when a stdio buffer is flushed. Run inside the container, with the
# same environment as the simulation (sim_noc.sh).
set -u
out=${1:?usage: log_ft.sh <out_file> [topic]}
topic=${2:-/noc/ee_ft}
here=$(cd "$(dirname "$0")" && pwd)

command -v gz >/dev/null 2>&1 || { echo "log_ft.sh: gz not found in PATH" >&2; exit 127; }
if command -v stdbuf >/dev/null 2>&1; then lb="stdbuf -oL"; else lb=""; fi

trap 'kill 0' INT TERM
printf '# log_ft.sh topic=%s started=%s stdbuf=%s\n' "$topic" "$(date +%s.%N)" "${lb:-none}" > "$out"
$lb gz topic -e -t "$topic" --json-output | python3 -u "$here/ft_stamp.py" >> "$out"
