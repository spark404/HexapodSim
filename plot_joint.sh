#!/bin/bash
# Usage: ./plot_joint.sh [joint_name]
# Example: ./plot_joint.sh leg_fr_servo_1
#
# Logs commanded vs actual position for one joint and plots live in gnuplot.
# Requires: gz (Gazebo CLI), python3, gnuplot

JOINT=${1:-leg_fr_servo_1}
CSV=/tmp/joint_${JOINT}.csv

echo "Tracking: $JOINT  →  $CSV"
echo "# time_s cmd actual" > "$CSV"

# Start the Python logger in background
python3 /Users/htrippaers/Projects/Personal/HexapodSim/log_joint.py "$JOINT" "$CSV" &
LOGGER_PID=$!

cleanup() {
    echo "Stopping..."
    kill $LOGGER_PID 2>/dev/null
    exit 0
}
trap cleanup INT TERM

# Give logger a moment to start writing
sleep 0.5

# gnuplot live loop
gnuplot << 'EOF'
joint = system("echo $JOINT")   # not available inside heredoc — use ARG1 instead
EOF

# Re-run with the variable available to gnuplot
JOINT="$JOINT" CSV="$CSV" gnuplot << 'GPEOF'
joint = system("echo $JOINT")
csv   = system("echo $CSV")

set terminal qt size 900,500 title "Joint position: ".joint
set xlabel "Time (s)"
set ylabel "Position (rad)"
set grid
set key top right
set style line 1 lw 2 lc rgb "#e74c3c"
set style line 2 lw 2 lc rgb "#3498db"

while (1) {
    plot csv using 1:2 with lines ls 1 title "commanded", \
         csv using 1:3 with lines ls 2 title "actual"
    pause 0.2
}
GPEOF

cleanup
