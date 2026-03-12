"""
Smoke test — verifies the full pipeline from config → dispatcher → mock robot.
Run with: python scripts/smoke_test.py
No hardware or display required.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from surgical_test.sim import get_robot
from surgical_test.command_dispatcher import CommandDispatcher

print("--- smoke test: mock robot ---")

robot = get_robot("surgical.conf")   # mode=mock by default
dispatcher = CommandDispatcher(robot=robot)

test_commands = [
    {"command": "move_right",   "magnitude": "small"},
    {"command": "move_up",      "magnitude": "medium"},
    {"command": "move_forward", "magnitude": "tiny"},
    {"command": "grip"},
    {"command": "hold_wider"},
    {"command": "hold_tighter"},
    {"command": "go_home"},
    {"command": "open_hand"},
    {"command": "unknown_cmd"},   # should warn, not crash
    {"command": "stop"},
]

failed = []
for cmd in test_commands:
    try:
        dispatcher.dispatch(cmd)
    except Exception as e:
        failed.append((cmd, str(e)))

if failed:
    print("\nFAILED:")
    for cmd, err in failed:
        print(f"  {cmd} -> {err}")
    sys.exit(1)
else:
    print("\nAll commands dispatched without exceptions.")
    print("Smoke test passed.")
    sys.exit(0)
