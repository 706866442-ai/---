# Encoder probe flash and initial readback — 2026-10-16

- Target: STM32F103C8T6, connected through ST-LINK V2.
- Environment: PlatformIO `encoder_probe`; built image Flash 16,896 B, RAM 1,628 B.
- Upload result: `Verified OK`, target reset after programming. This replaced the firmware previously on this board. The probe drives TB6612 STBY/PWM/direction pins low and has no motor-start logic.
- ST-LINK target voltage observed before programming: 3.225 V. This is the MCU target supply reading, not a measurement of encoder output levels.
- Read-only SWD snapshot of `g_encoderProbe` about 45 s after boot: signature `0x454E4331`, sequence 4534, uptime 45,349 ms, E1/E2 raw counts 0/0, deltas 0/0, invalid transitions 0/0, E1/E2 AB states 0/1, `motorEnabled=0`.
- No wheel was moved during this snapshot. Zero counts do not establish encoder failure or success.
- No USB-TTL serial device was detected on the PC; UART display at PA2 has not been observed. Readback used ST-LINK memory reads without issuing reset/halt/GPIO writes after upload.
- Next measurement: with the vehicle safely supported and motor drive disabled, turn one wheel by hand and read the probe again; then repeat for the other wheel. This identifies which channel belongs to each side. Do not start PID based on this initial snapshot.