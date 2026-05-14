# Manideep Sai C
# Reg.no 23BCE0737

from flask import Flask, request, jsonify
from flask_cors import CORS
import math
import re
import serial
import time
import threading

app = Flask(__name__)
CORS(app)

COM_PORT = "COM6"
BAUD_RATE = 9600
TURN_TIME = 0.85
FWD_TIME = 1.0
CONTINUOUS_MODE = False
POSITION_EPSILON = 0.05

serial_lock = threading.Lock()

rover = None
connected = False
path_history = []
command_log = []
pos_x, pos_y = 0.0, 0.0
direction = 0.0
active_command = None
active_started_at = None
dir_names = ["North", "East", "South", "West"]


def get_json_data():
    return request.get_json(silent=True) or {}


def direction_name():
    return dir_names[int(round(direction)) % 4]


def default_duration(cmd):
    if cmd in ("L", "R"):
        return TURN_TIME
    if cmd in ("F", "B"):
        return FWD_TIME
    return 0.0


def safe_float(val, default_val=0.0):
    try:
        if val is None:
            return default_val
        return float(val)
    except (ValueError, TypeError):
        return default_val


def connect_rover():
    global rover, connected
    try:
        if rover and rover.is_open:
            try:
                rover.close()
            except Exception:
                pass
        rover = serial.Serial(COM_PORT, BAUD_RATE, timeout=3, write_timeout=2)
        time.sleep(2)
        connected = True
        return True
    except Exception:
        connected = False
        rover = None
        return False


def send_command(cmd):
    global rover, connected
    if not connected or rover is None:
        return False
    with serial_lock:
        try:
            rover.write(cmd.encode())
            rover.flush()
            return True
        except Exception:
            connected = False
            if rover:
                try:
                    rover.close()
                except Exception:
                    pass
            rover = None
            return False


def track_movement(cmd, duration=None, record=True):
    global pos_x, pos_y, direction, path_history, command_log
    duration = safe_float(duration if duration is not None else default_duration(cmd), 0.0)

    # Always update position and direction so return logic stays in sync
    if cmd in ("F", "B"):
        travel_units = duration / FWD_TIME if FWD_TIME else 0.0
        if cmd == "B":
            travel_units *= -1
        radians = math.radians(direction * 90.0)
        pos_x += math.sin(radians) * travel_units
        pos_y += math.cos(radians) * travel_units
    elif cmd == "L":
        direction = (direction - (duration / TURN_TIME if TURN_TIME else 0.0)) % 4
    elif cmd == "R":
        direction = (direction + (duration / TURN_TIME if TURN_TIME else 0.0)) % 4

    if record and cmd in ("F", "B"):
        path_history.append({"x": round(pos_x, 3), "y": round(pos_y, 3), "time": round(duration, 3)})
        
    if record:
        command_log.append({"cmd": cmd, "ms": round(duration * 1000)})


def timed_move(cmd, duration, record=True):
    duration = safe_float(duration, 0.0)
    if duration < 0:
        duration = 0.0
    if not send_command(cmd):
        return False
    time.sleep(duration)
    if not send_command("S"):
        return False
    time.sleep(0.2)
    track_movement(cmd, duration, record=record)
    return True


def stop_active_move(record=True):
    global active_command, active_started_at
    if active_command is None or active_started_at is None:
        send_command("S")
        return {
            "status": "success",
            "message": "Already stopped",
            "duration": 0.0,
        }

    cmd = active_command
    duration = max(time.monotonic() - active_started_at, 0.0)
    if not send_command("S"):
        active_command = None
        active_started_at = None
        return {"status": "error", "message": "Stop failed"}

    if record:
        track_movement(cmd, duration, record=True)

    active_command = None
    active_started_at = None
    return {
        "status": "success",
        "message": "Stopped",
        "command": cmd,
        "duration": round(duration, 4),
    }


def face_direction(target_dir):
    diff = (target_dir - direction + 4.0) % 4.0
    if diff > 2.0:
        diff -= 4.0
    if abs(diff) < 0.01:
        return True
    turn_cmd = "R" if diff > 0 else "L"
    turn_duration = abs(diff) * TURN_TIME
    return timed_move(turn_cmd, turn_duration, record=False)


def min_turn_cost(from_dir, to_dir):
    """Minimum turn amount (in direction units 0-2) between two cardinal headings."""
    diff = abs((to_dir - from_dir + 4.0) % 4.0)
    return min(diff, 4.0 - diff)


def execute_exact_return():
    """Retraces the recorded waypoints in reverse order using strict tape playback."""
    global path_history, command_log, pos_x, pos_y, direction
    if active_command is not None:
        stop_result = stop_active_move(record=True)
        if stop_result["status"] == "error":
            return stop_result

    if not command_log:
        return {"status": "success", "message": "Re-aligned to origin.", **current_state()}

    # Exact Return: Play the tape mathematically in reverse
    reverse_log = list(reversed(command_log))
    
    inverse_cmd = {
        "F": "B",
        "B": "F",
        "R": "L",
        "L": "R"
    }

    for action in reverse_log:
        cmd = action["cmd"]
        if cmd not in inverse_cmd:
            continue
            
        inv_cmd = inverse_cmd[cmd]
        move_duration = action["ms"] / 1000.0
        
        if move_duration <= 0.01:
            continue

        if not timed_move(inv_cmd, move_duration, record=False):
            return {"status": "error", "message": "Return maneuver failed", **current_state()}

    path_history = []
    command_log = []
    pos_x, pos_y, direction = 0.0, 0.0, 0.0
    return {"status": "success", "message": "Return complete", **current_state()}


def execute_shortest_return():
    """
    Returns to origin minimizing turns by sliding along axis, allowing reverse driving.
    """
    global path_history, command_log, pos_x, pos_y, direction

    if active_command is not None:
        stop_result = stop_active_move(record=True)
        if stop_result["status"] == "error":
            return stop_result

    if abs(pos_x) < POSITION_EPSILON and abs(pos_y) < POSITION_EPSILON:
        path_history = []
        command_log = []
        return {"status": "success", "message": "Re-aligned to origin."}

    # Identify needed traversals
    traversals = []
    if abs(pos_y) >= POSITION_EPSILON:
        target_dir = 2.0 if pos_y > 0 else 0.0
        traversals.append({ "dir": target_dir, "dist": abs(pos_y), "axis": "Y" })
    
    if abs(pos_x) >= POSITION_EPSILON:
        target_dir = 3.0 if pos_x > 0 else 1.0
        traversals.append({ "dir": target_dir, "dist": abs(pos_x), "axis": "X" })

    # Sort traversals: do the axis we are currently parallel to FIRST
    def is_parallel(axis_name, heading):
        h = round(heading) % 4
        if axis_name == "Y" and h in (0, 2): return True
        if axis_name == "X" and h in (1, 3): return True
        return False

    if len(traversals) == 2:
        if is_parallel(traversals[1]["axis"], direction):
            traversals = [traversals[1], traversals[0]]

    for t in traversals:
        tgt_dir = t["dir"]
        dist = t["dist"]
        
        diff = (tgt_dir - direction + 4.0) % 4.0
        if diff > 2.0:
            diff -= 4.0
            
        move_cmd = "F"
        
        # If facing 180 away, go backward!
        if abs(abs(diff) - 2.0) < 0.01:
            move_cmd = "B"
        elif abs(diff) > 0.01:
            # Requires 90 degree turn
            turn_cmd = "R" if diff > 0 else "L"
            turn_duration = abs(diff) * TURN_TIME
            if not timed_move(turn_cmd, turn_duration, record=False):
                return {"status": "error", "message": "Turn failed", **current_state()}
            move_cmd = "F"
            
        move_duration = dist * FWD_TIME
        if not timed_move(move_cmd, move_duration, record=False):
            return {"status": "error", "message": "Move failed", **current_state()}
            
        if t["axis"] == "Y":
            pos_y = 0.0
        else:
            pos_x = 0.0

    path_history = []
    command_log = []
    return {"status": "success", "message": "Return complete", **current_state()}


def parse_sequence(seq):
    normalized = re.sub(r"[^A-Z0-9]+", "", seq.upper())
    tokens = re.findall(r"(\d*)([FBLRUDSC])", normalized)
    moves = []
    for count_text, cmd in tokens:
        count = int(count_text) if count_text else 1
        moves.extend(cmd for _ in range(max(count, 1)))
    return moves


def current_state():
    return {
        "connected": connected,
        "position": [round(pos_x, 3), round(pos_y, 3)],
        "direction": direction_name(),
        "heading_turns": round(direction, 3),
        "path_length": len(path_history),
        "command_log": command_log,
        "turn_time": TURN_TIME,
        "fwd_time": FWD_TIME,
        "continuous_mode": CONTINUOUS_MODE,
        "active_command": active_command,
    }


@app.route("/connect", methods=["POST"])
def api_connect():
    if connect_rover():
        return jsonify({"status": "success", "message": "Connected", **current_state()})
    return jsonify({"status": "error", "message": "Connect HC-05 in Windows Bluetooth settings first"}), 400


@app.route("/disconnect", methods=["POST"])
def api_disconnect():
    global rover, connected, active_command, active_started_at
    if rover:
        try:
            rover.close()
        except:
            pass
    rover = None
    connected = False
    active_command = None
    active_started_at = None
    return jsonify({"status": "success", "message": "Disconnected", **current_state()})


@app.route("/status", methods=["GET"])
def api_status():
    return jsonify(current_state())


@app.route("/send", methods=["POST"])
def api_send():
    data = get_json_data()
    cmd = data.get("command", "").upper()
    if not cmd:
        return jsonify({"status": "error", "message": "No command"}), 400

    if cmd == "S":
        result = stop_active_move(record=True)
        status_code = 200 if result["status"] == "success" else 400
        return jsonify({**result, **current_state()}), status_code

    if cmd in ["F", "B", "L", "R"]:
        duration = default_duration(cmd)
        if timed_move(cmd, duration, record=True):
            return jsonify({"status": "success", "command": cmd, "duration": duration, **current_state()})
        return jsonify({"status": "error", "message": "Command failed", **current_state()}), 400

    if cmd in ["A", "M"] and send_command(cmd):
        return jsonify({"status": "success", "command": cmd, **current_state()})

    return jsonify({"status": "error", "message": "Command failed", **current_state()}), 400


@app.route("/move", methods=["POST"])
def api_move():
    global active_command, active_started_at
    data = get_json_data()
    cmd = data.get("command", "").upper()
    continuous = bool(data.get("continuous", False) or CONTINUOUS_MODE)
    duration = safe_float(data.get("duration", default_duration(cmd)), 0.0)

    if cmd not in ["F", "B", "L", "R"]:
        return jsonify({"status": "error", "message": "Move failed", **current_state()}), 400

    if continuous:
        if active_command == cmd:
            return jsonify({"status": "success", "command": cmd, "continuous": True, **current_state()})
        if active_command is not None:
            stop_result = stop_active_move(record=True)
            if stop_result["status"] == "error":
                return jsonify(stop_result), 400
        if send_command(cmd):
            active_command = cmd
            active_started_at = time.monotonic()
            return jsonify({"status": "success", "command": cmd, "continuous": True, **current_state()})
        return jsonify({"status": "error", "message": "Move failed", **current_state()}), 400

    if timed_move(cmd, duration, record=True):
        return jsonify({"status": "success", "command": cmd, "continuous": False, "duration": duration, **current_state()})
    return jsonify({"status": "error", "message": "Move failed", **current_state()}), 400


@app.route("/stop", methods=["POST"])
def api_stop():
    result = stop_active_move(record=True)
    status_code = 200 if result["status"] == "success" else 400
    return jsonify({**result, **current_state()}), status_code


@app.route("/sequence", methods=["POST"])
def api_sequence():
    global path_history, pos_x, pos_y, direction
    data = get_json_data()
    seq = data.get("sequence", "")
    moves = parse_sequence(seq)
    if not moves:
        return jsonify({"status": "error", "message": "No valid moves"}), 400

    if active_command is not None:
        stop_result = stop_active_move(record=True)
        if stop_result["status"] == "error":
            return jsonify(stop_result), 400

    for cmd in moves:
        if cmd == "U":
            result = execute_exact_return()
            if result["status"] == "error":
                return jsonify(result), 400
        elif cmd == "D":
            result = execute_shortest_return()
            if result["status"] == "error":
                return jsonify(result), 400
        elif cmd == "C":
            path_history = []
            pos_x, pos_y, direction = 0.0, 0.0, 0.0
        elif cmd == "S":
            send_command("S")
        else:
            duration = default_duration(cmd)
            if not timed_move(cmd, duration, record=True):
                return jsonify({"status": "error", "message": "Sequence failed", **current_state()}), 400
    return jsonify({"status": "success", "moves": len(moves), **current_state()})


@app.route("/return/exact", methods=["POST"])
def api_exact_return():
    result = execute_exact_return()
    status_code = 200 if result["status"] == "success" else 400
    return jsonify({**result, **current_state()}), status_code


@app.route("/return/shortest", methods=["POST"])
def api_shortest_return():
    result = execute_shortest_return()
    status_code = 200 if result["status"] == "success" else 400
    return jsonify({**result, **current_state()}), status_code


@app.route("/clear", methods=["POST"])
def api_clear():
    global path_history, command_log, pos_x, pos_y, direction, active_command, active_started_at
    send_command("S")
    active_command = None
    active_started_at = None
    path_history = []
    command_log = []
    pos_x, pos_y, direction = 0.0, 0.0, 0.0
    return jsonify({"status": "success", "message": "Path cleared", **current_state()})


@app.route("/calibrate", methods=["POST"])
def api_calibrate():
    global TURN_TIME, FWD_TIME, CONTINUOUS_MODE
    data = get_json_data()
    if "turn_time" in data:
        TURN_TIME = float(data["turn_time"])
    if "fwd_time" in data:
        FWD_TIME = float(data["fwd_time"])
    if "continuous_mode" in data:
        CONTINUOUS_MODE = bool(data["continuous_mode"])
    return jsonify({"status": "success", "turn_time": TURN_TIME, "fwd_time": FWD_TIME, "continuous_mode": CONTINUOUS_MODE, **current_state()})


@app.route("/test", methods=["POST"])
def api_test():
    data = get_json_data()
    cmd = (data.get("direction") or data.get("command") or "L").upper()
    duration = safe_float(data.get("duration", default_duration(cmd)), 0.0)
    if cmd not in ["F", "B", "L", "R"]:
        return jsonify({"status": "error", "message": "Invalid test command", **current_state()}), 400
    if timed_move(cmd, duration, record=False):
        return jsonify({"status": "success", "message": "Test complete", **current_state()})
    return jsonify({"status": "error", "message": "Test failed", **current_state()}), 400


if __name__ == "__main__":
    print("Smart Rover Server - http://localhost:5000")
    print("1. Pair HC-05 in Windows Bluetooth (PIN: 1234)")
    print("2. Connect HC-05 in Bluetooth Settings")
    print("3. Open index.html and click Connect")
    print("-" * 50)
    app.run(host="0.0.0.0", port=5000, debug=False)