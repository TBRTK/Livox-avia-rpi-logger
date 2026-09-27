#!/bin/bash

set -e

source ~/ws_livox/install/setup.bash

BASE="/data/livox_bags"
LOGDIR="$BASE/logs"

mkdir -p "$BASE" "$LOGDIR"

# --------------------------------------------------
# Sjekk at et annet rosbag-opptak ikke allerede går
# --------------------------------------------------

if pgrep -f "ros2 bag record" >/dev/null; then
    echo "FEIL: Et rosbag-opptak kjører allerede."
    echo
    pgrep -af "ros2 bag record"
    exit 1
fi

# --------------------------------------------------
# Diskplass med 10 GiB reserve
# --------------------------------------------------

RESERVE_GIB=10
MIN_START_GIB=15

FREE_KB=$(df -Pk "$BASE" | awk 'NR==2 {print $4}')
FREE_GIB=$(awk -v kb="$FREE_KB" 'BEGIN {printf "%.1f", kb/1024/1024}')

USABLE_GIB=$(awk -v free="$FREE_GIB" -v reserve="$RESERVE_GIB"     'BEGIN {usable=free-reserve; if (usable < 0) usable=0; printf "%.1f", usable}')

EST_HOURS=$(awk -v usable="$USABLE_GIB"     'BEGIN {printf "%.1f", usable/16.4}')

echo "========================================"
echo " Livox feltopptak"
echo "========================================"
echo "Ledig diskplass:          $FREE_GIB GiB"
echo "Reservert buffer:         $RESERVE_GIB GiB"
echo "Tilgjengelig for opptak:  $USABLE_GIB GiB"
echo "Estimert opptakstid:      ca. $EST_HOURS timer"
echo

if awk -v free="$FREE_GIB" -v min="$MIN_START_GIB"     'BEGIN {exit !(free < min)}'; then
    echo "FEIL: For lite ledig diskplass."
    echo "Minimum $MIN_START_GIB GiB må være ledig for å starte."
    exit 1
fi

# --------------------------------------------------
# Livox-driver
# --------------------------------------------------

echo "Kontrollerer Livox-driver..."

if ! pgrep -f "livox_ros2_driver_node" >/dev/null; then
    echo "Livox-driver kjører ikke - starter den."

    DRIVER_LOG="$LOGDIR/livox_driver_$(date +"%Y-%m-%d_%H-%M-%S").log"

    nohup ros2 launch livox_ros2_driver livox_lidar_msg_launch.py \
        > "$DRIVER_LOG" 2>&1 &

    echo "Driver startet."
    echo "Logg: $DRIVER_LOG"
    echo "Venter på at Avia initialiseres..."

    READY=0

    for i in $(seq 1 60); do
        if grep -q "Lidar start sample success" "$DRIVER_LOG" 2>/dev/null; then
            READY=1
            break
        fi
        sleep 1
    done

    if [ "$READY" -ne 1 ]; then
        echo
        echo "FEIL: Livox-driver ble ikke klar innen 60 sekunder."
        echo "Se logg:"
        echo "$DRIVER_LOG"
        exit 1
    fi

    echo "Avia initialisert."
else
    echo "Livox-driver kjører allerede."
fi

# --------------------------------------------------
# Kontroller faktiske data
# --------------------------------------------------

echo
echo "Kontrollerer LiDAR-data..."

if ! timeout 20 ros2 topic echo /livox/lidar --once --field point_num \
        >/dev/null 2>&1; then
    echo
    echo "FEIL: Ingen LiDAR-data mottatt."
    exit 1
fi

echo "LiDAR OK."

echo "Kontrollerer IMU..."

if ! timeout 15 ros2 topic echo /livox/imu --once --field header.stamp \
        >/dev/null 2>&1; then
    echo
    echo "FEIL: Ingen IMU-data mottatt."
    exit 1
fi

echo "IMU OK."

# --------------------------------------------------
# Start monitor
# --------------------------------------------------

STAMP=$(date +"%Y-%m-%d_%H-%M-%S")
MONITOR_LOG="$LOGDIR/livox_monitor_$STAMP.log"

echo
echo "Starter Livox monitor v1.0..."
stdbuf -oL -eL ros2 run livox_monitor livox_monitor_node \
    > "$MONITOR_LOG" 2>&1 &
MONITOR_PID=$!

sleep 2

if ! kill -0 "$MONITOR_PID" 2>/dev/null; then
    echo
    echo "FEIL: Livox monitor startet ikke."
    echo "Se logg:"
    echo "$MONITOR_LOG"
    exit 1
fi

echo "Monitor startet."
echo "Monitorlogg: $MONITOR_LOG"

# --------------------------------------------------
# Start opptak
# --------------------------------------------------

OUT="$BASE/avia_$STAMP"

echo
echo "========================================"
echo " Livox klar - starter opptak"
echo "========================================"
echo "Lagrer til:"
echo "$OUT"
echo
echo "Ledig plass: $FREE_GIB GiB"
echo "Ca. $EST_HOURS timer kapasitet"
echo
echo "Stopp opptaket med Ctrl+C"
echo "========================================"
echo

# --------------------------------------------------
# Opptak med automatisk diskovervåkning
# --------------------------------------------------

ros2 bag record \
    -o "$OUT" \
    /livox/lidar \
    /livox/imu &

BAG_PID=$!

stop_recording() {
    if kill -0 "$BAG_PID" 2>/dev/null; then
        echo
        echo "Stopper opptaket kontrollert..."
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi

    if [ -n "${MONITOR_PID:-}" ] && kill -0 "$MONITOR_PID" 2>/dev/null; then
        echo "Stopper Livox monitor..."
        kill -INT "$MONITOR_PID" 2>/dev/null || true
        wait "$MONITOR_PID" 2>/dev/null || true
    fi
}

trap 'set +e; stop_recording' INT TERM

while kill -0 "$BAG_PID" 2>/dev/null; do
    sleep 30 || true

    FREE_NOW_KB=$(df -Pk "$BASE" | awk 'NR==2 {print $4}')
    FREE_NOW_GIB=$(awk -v kb="$FREE_NOW_KB" 'BEGIN {printf "%.2f", kb/1024/1024}')

    if awk -v free="$FREE_NOW_GIB" -v reserve="$RESERVE_GIB"         'BEGIN {exit !(free <= reserve)}'; then

        echo
        echo "========================================"
        echo " DISKRESERVE NÅDD"
        echo "========================================"
        echo "Ledig plass: $FREE_NOW_GIB GiB"
        echo "Minimum reserve: $RESERVE_GIB GiB"
        echo "Opptaket stoppes automatisk."
        echo

        stop_recording
        break
    fi
done

wait "$BAG_PID" 2>/dev/null || true

if [ -n "${MONITOR_PID:-}" ] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill -INT "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
fi

trap - INT TERM

echo
echo "Opptak avsluttet."
echo "Monitorlogg: $MONITOR_LOG"

if grep -Eq "QUALITY[[:space:]]*:[[:space:]]*(WARNING|DEGRADED)" "$MONITOR_LOG" 2>/dev/null; then
    echo "MERK: Monitor registrerte WARNING/DEGRADED under opptaket."
    echo "Se: $MONITOR_LOG"
else
    echo "Monitor: ingen QUALITY WARNING/DEGRADED registrert."
fi
