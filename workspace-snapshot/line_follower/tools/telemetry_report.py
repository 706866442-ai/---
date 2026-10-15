"""Convert captured T CSV lines to a normalized CSV and a concise diagnostic report.

No motor commands, no automatic gain changes. Output directory must be explicit.
"""
import argparse
import csv
from pathlib import Path

FIELDS = ('ms state mode phase gray gray_valid line_error e1 e2 invalid '
          'left_target right_target left_speed right_speed left_pwm right_pwm '
          'fault dropped gray_age_ms speed_valid').split()

def parse_lines(lines):
    rows, rejected = [], 0
    for line in lines:
        if not line.startswith('T,'):
            continue
        parts = line.strip().split(',')[1:]
        try:
            if len(parts) != len(FIELDS):
                raise ValueError('wrong field count')
            values = [int(v) for v in parts]
            row = dict(zip(FIELDS, values))
            if not 0 <= row['ms'] <= 0xffffffff or row['gray_valid'] not in (0, 1) or row['speed_valid'] not in (0, 1):
                raise ValueError('invalid flags/time')
            rows.append(row)
        except ValueError:
            rejected += 1
    return rows, rejected

def report(rows, rejected):
    if not rows:
        raise ValueError('no valid telemetry records; check firmware format and UART mode')
    gaps = [((b['ms'] - a['ms']) & 0xffffffff) for a, b in zip(rows, rows[1:])]
    valid = [r for r in rows if r['gray_valid'] and r['gray'] not in (0, 255)]
    speed = [r for r in rows if r['speed_valid']]
    return '\n'.join([
        '# 小车日志分析', '',
        f'- 有效记录：{len(rows)}；损坏的 T 行：{rejected}。',
        f'- 最大日志间隔：{max(gaps, default=0)} ms（日志约 100 ms 一次，不等于控制周期）。',
        f'- 最大有效灰度误差绝对值：{max((abs(r["line_error"]) for r in valid), default=0)}。',
        f'- 内环有效记录：{len(speed)}；最大左右速度误差：'
        f'{max((abs(r["left_target"]-r["left_speed"]) for r in speed), default=0)} / '
        f'{max((abs(r["right_target"]-r["right_speed"]) for r in speed), default=0)} mm/s。',
        f'- 出现的故障码：{sorted({r["fault"] for r in rows if r["fault"]})}。',
        f'- 队列累计丢弃记录最大值：{max(r["dropped"] for r in rows)}。', '',
        '速度值依赖已标定的编码器比例；本报告不能证明实车通过验收，也不会自动修改 PID。',
    ]) + '\n'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows, rejected = parse_lines(args.input.read_text(encoding='utf-8', errors='replace').splitlines())
    summary = report(rows, rejected)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'telemetry.csv').open('w', encoding='utf-8-sig', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader(); writer.writerows(rows)
    (args.output / '诊断报告.md').write_text(summary, encoding='utf-8')
    print(summary)

if __name__ == '__main__':
    main()
