<!-- consult selectively — grep, never read in full -->
# Error Log

## 2026-03-21 - DAC output never worked: channel index vs GPIO pin number
**What happened:** `KM_ACT_SetOutput` for throttle/brake never produced DAC output. GPIO 25 always read ~0.12V (noise). The full Orin→serial→ESP32 pipeline appeared to work (frames received, no errors logged), but the actuator output was silently failing.
**Root cause:** `KM_ACT_Init(ACT_ACCEL)` set `dacChannel = 0` (a channel index). `KM_GPIO_WriteDAC` expected a `gpio_num_t` (25 for ACC, 26 for BRAKE) and compared with `PIN_CMD_ACC` (25). Since `0 != 25`, it returned `ESP_ERR_INVALID_ARG` silently. The return value was never checked.
**How we found it:** Instead of debugging the full pipeline (serial protocol, ROS topics, state machine, etc.), we wrote a minimal test: hardcode `dac_output_voltage(DAC_CHAN_0, 128)` directly in `main.c`, bypassing all abstraction. This immediately produced 1.65V on GPIO 25, proving the hardware works and the bug was in our abstraction layer. This binary search approach — test at the boundary to determine if the error is upstream or downstream — makes deterministic progress regardless of the test result.
**Fix:** Changed `dacChannel` from channel indices (0/1) to GPIO pin numbers (PIN_CMD_ACC/PIN_CMD_BRAKE).
**Prevention added:**
- Rule: **When debugging a pipeline, don't test end-to-end first. Find a test that splits the pipeline in half — the result tells you which half has the bug, making progress no matter what.** Hardcoding an output at the hardware boundary is the fastest way to isolate software vs hardware issues.
- Rule: **Always check return values from hardware write functions.** `KM_GPIO_WriteDAC` returned an error that was silently ignored.
