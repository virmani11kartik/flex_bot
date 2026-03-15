import pygame
import serial
import time
import threading
import json
import os

# ═══════════════════════════════════════════════════════
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200

BTN_STEP_CW     = 1
BTN_STEP_CCW    = 0
BTN_SPEED_UP    = 10
BTN_SPEED_DOWN  = 9
BTN_ACT_EXTEND  = 2
BTN_ACT_RETRACT = 3
BTN_AUTO_TOGGLE = 6   # ← new: toggle autonomous mode

SPEED_DEFAULT = 500
SPEED_MIN     = 300
SPEED_MAX     = 3000
SPEED_STEP    = 300

PRINT_BUTTONS = True

# ── Autonomous tuning ─────────────────────────────────
CENTER_LOW    = 0.55
CENTER_HIGH   = 0.85
DEADBAND      = 0.02
MOVE_DURATION = 2.0   # seconds per nudge
# ═══════════════════════════════════════════════════════

# ── Shared state written by OCR process, read here ────
AUTO_STATE_FILE = os.path.expanduser("~/Desktop/autonomous_state.json")

def read_autonomous_state():
    try:
        with open(AUTO_STATE_FILE, "r") as f:
            return json.load(f)
    except:
        return {
            "clusters": [], 
            "roi_height": 100,
            "has_target": False,
            "target_center_y": 0,
            "alignment_error": 999,
            "is_aligned": False
        }
    
auto_mode = False   # toggled by BTN_AUTO_TOGGLE


def find_serial_port():
    import serial.tools.list_ports
    print("\nAvailable serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}  —  {p.description}")
    print()


def compute_avg_y(clusters, roi_height):
    """Return normalized average Y of all text boxes, or None if no detections."""
    all_ys = []
    for cluster in clusters:
        for box, text, conf, cx, cy in cluster["items"]:
            all_ys.append(cy / roi_height)
    return sum(all_ys) / len(all_ys) if all_ys else None


def autonomous_tick(ser, act_tracker):
    """
    Continuous actuator controller.
    Moves camera until detected rectangle is aligned with ROI center.
    """
    state = read_autonomous_state()
    
    # Debug print
    print(f"DEBUG - has_target: {state.get('has_target', False)}")
    print(f"DEBUG - is_aligned: {state.get('is_aligned', False)}")
    print(f"DEBUG - alignment_error: {state.get('alignment_error', 999)}")
    print(f"DEBUG - target_center_y: {state.get('target_center_y', 0)}")
    print(f"DEBUG - roi_center_y: {state.get('roi_center_y', 0)}")
    
    # If no target detected, stop
    if not state.get("has_target", False):
        _send(ser, "CMD:ACT,0", act_tracker)
        print("No target rectangle detected - stopping")
        return
    
    # If already aligned, stop
    if state.get("is_aligned", False):
        _send(ser, "CMD:ACT,0", act_tracker)
        print(f"Target aligned (error: {state.get('alignment_error', 0):.1f}px) - stopping")
        return
    
    # Get target position and ROI center
    target_y = state.get("target_center_y", 0)
    roi_center_y = state.get("roi_center_y", 0)
    
    if roi_center_y == 0:
        print("No ROI center data - stopping")
        _send(ser, "CMD:ACT,0", act_tracker)
        return
    
    # Calculate error: positive means target is below ROI center (need to RAISE)
    # Negative means target is above ROI center (need to LOWER)
    error = target_y - roi_center_y
    print(f"Error: {error:.1f}px (target_y={target_y}, roi_center={roi_center_y})")
    
    # Deadband to prevent jitter
    if abs(error) < 25:  # Same as ALIGNMENT_TOLERANCE
        _send(ser, "CMD:ACT,0", act_tracker)
        print("Within deadband - stopping")
        return
    
    # Move in correct direction
    if error > 0:
        # Target is BELOW ROI center → need to RAISE camera
        print(f"Target below center → RAISING camera")
        _send(ser, "CMD:ACT,-1", act_tracker)  # Extend = raise
    else:
        # Target is ABOVE ROI center → need to LOWER camera
        print(f"Target above center → LOWERING camera")
        _send(ser, "CMD:ACT,1", act_tracker)  # Retract = lower

def _send(ser, cmd, tracker):
    if cmd != tracker[0]:
        ser.write((cmd + "\n").encode())
        print(f"-> {cmd}")
        tracker[0] = cmd
        time.sleep(0.02)
        resp = ser.read(64).decode(errors="replace").strip()
        if resp:
            print(f"   ESP32: {resp}")


def send_speed(ser, spd):
    msg = f"CMD:SPEED,{spd}"
    ser.write((msg + "\n").encode())
    print(f"-> {msg}")
    time.sleep(0.02)
    resp = ser.read(64).decode(errors="replace").strip()
    if resp:
        print(f"   ESP32: {resp}")


def main():
    global auto_mode
    find_serial_port()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Opened: {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"ERROR: {e}"); return

    time.sleep(1.5)
    startup = ser.read(200).decode(errors="replace").strip()
    if startup:
        print(f"ESP32: {startup}")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: No joystick found."); ser.close(); return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Controller: {joy.get_name()}  Buttons:{joy.get_numbuttons()}  Axes:{joy.get_numaxes()}")

    current_speed = SPEED_DEFAULT
    step_tracker  = [""]
    act_tracker   = [""]

    send_speed(ser, current_speed)

    print(f"\nControls:")
    print(f"  Btn {BTN_STEP_CW}/{BTN_STEP_CCW}       → Stepper CW/CCW (hold)")
    print(f"  Btn {BTN_ACT_EXTEND}/{BTN_ACT_RETRACT}     → Actuator extend/retract (hold) [manual only]")
    print(f"  Btn {BTN_SPEED_UP}/{BTN_SPEED_DOWN}       → Speed +/- (tap)")
    print(f"  Btn {BTN_AUTO_TOGGLE}         → Toggle autonomous mode")
    print()

    try:
        clock = pygame.time.Clock()
        while True:
            for event in pygame.event.get():
                if PRINT_BUTTONS:
                    if event.type == pygame.JOYBUTTONDOWN:
                        print(f"  [DEBUG] Button {event.button} PRESSED")
                    if event.type == pygame.JOYBUTTONUP:
                        print(f"  [DEBUG] Button {event.button} released")

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_SPEED_UP:
                        current_speed = min(current_speed + SPEED_STEP, SPEED_MAX)
                        send_speed(ser, current_speed)
                    elif event.button == BTN_SPEED_DOWN:
                        current_speed = max(current_speed - SPEED_STEP, SPEED_MIN)
                        send_speed(ser, current_speed)
                    elif event.button == BTN_AUTO_TOGGLE:
                        auto_mode = not auto_mode
                        print(f"\n[Mode] {'AUTONOMOUS 🤖' if auto_mode else 'MANUAL 🕹️'}\n")
                        if not auto_mode:
                            _send(ser, "CMD:ACT,0", act_tracker)  # stop on exit

            def btn(i): return joy.get_button(i) if i < joy.get_numbuttons() else 0

            # Stepper always works regardless of mode
            if   btn(BTN_STEP_CW)  and not btn(BTN_STEP_CCW):  _send(ser, "CMD:STEP,1",  step_tracker)
            elif btn(BTN_STEP_CCW) and not btn(BTN_STEP_CW):   _send(ser, "CMD:STEP,-1", step_tracker)
            else:                                                _send(ser, "CMD:STEP,0",  step_tracker)

            # Actuator: manual buttons OR autonomous tick
            if auto_mode:
                autonomous_tick(ser, act_tracker)
            else:
                if   btn(BTN_ACT_EXTEND)  and not btn(BTN_ACT_RETRACT): _send(ser, "CMD:ACT,1",  act_tracker)
                elif btn(BTN_ACT_RETRACT) and not btn(BTN_ACT_EXTEND):  _send(ser, "CMD:ACT,-1", act_tracker)
                else:                                                     _send(ser, "CMD:ACT,0",  act_tracker)

            clock.tick(30)

    except KeyboardInterrupt:
        print("\nStopping...")
        ser.write(b"CMD:STEP,0\n")
        ser.write(b"CMD:ACT,0\n")

    finally:
        ser.close()
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()