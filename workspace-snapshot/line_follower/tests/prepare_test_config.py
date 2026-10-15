"""Create synthetic build/test inputs on D:. Never change the real calibration."""
from pathlib import Path
import re
import shutil
import sys

project = Path(__file__).resolve().parents[1]
destination = Path(sys.argv[1]).resolve()
camera = sys.argv[2] == 'vision'
destination.mkdir(parents=True, exist_ok=True)
for source in (project / 'include').glob('*.h'):
    shutil.copy2(source, destination / source.name)

def replace(text, name, value):
    text, count = re.subn(r'(constexpr\s+\w+\s+' + name + r'\s*=\s*)[^;]+;',
                          lambda m: m[1] + value + ';', text)
    assert count == 1, name
    return text

path = destination / 'RobotConfig.h'
text = path.read_text(encoding='utf-8')
for name in ['LINE_PID_ENABLED', 'SPEED_PID_ENABLED', 'ENCODER_INPUTS_ENABLED',
             'ENCODERS_VERIFIED', 'ROUTE_START_POSE_VERIFIED', 'PARKING_START_POSE_VERIFIED']:
    text = replace(text, name, 'true')
for name, value in dict(LINE_KP='0.01f', SPEED_KP='0.1f', LEFT_MM_PER_COUNT='1.0f',
                        RIGHT_MM_PER_COUNT='1.0f', SPEED_FEEDFORWARD_PWM='40.0f',
                        SPEED_BASE_MMPS='100.0f', SPEED_LINE_CORRECTION_MMPS='50.0f',
                        ROUTE_TURN_SIGN='1', BAY_SIDE='1', REVERSE_DEPTH_MM='120.0f',
                        PARALLEL_RADIUS_MM='150.0f', PARALLEL_ANGLE_DEG='30.0f').items():
    text = replace(text, name, value)
path.write_text('// SYNTHETIC TEST CONFIGURATION: NEVER UPLOAD\n' + text, encoding='utf-8')
path = destination / 'MissionConfig.h'
text = path.read_text(encoding='utf-8')
for name in ['COMPLETE_MISSIONS_ENABLED', 'VISION_PROTOCOL_VERIFIED', 'EXIT_POSE_VERIFIED']:
    text = replace(text, name, 'true')
text = replace(text, 'VISION_UART_ENABLED', 'true' if camera else 'false')
for name in ['REVERSE_APPROACH', 'PARALLEL_APPROACH']:
    text = replace(text, name, '{true, 0, 0, 0, 0, 100, 0, 0}')
for name, value in dict(EXIT_FORWARD_MM='100.0f', EXIT_FIRST_LEG_MM='500.0f',
                        EXIT_NEXT_POINT='2', EXIT_POINT_STEP='1').items():
    text = replace(text, name, value)
path.write_text('// SYNTHETIC TEST CONFIGURATION: NEVER UPLOAD\n' + text, encoding='utf-8')
