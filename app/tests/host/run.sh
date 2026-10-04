#!/usr/bin/env bash
# Build and run the host unit tests for the DaMiao protocol helpers.
set -euo pipefail
cd "$(dirname "$0")"
cc -std=c11 -Wall -Wextra -DDM_HOST_TEST -I../../src/dm test_dm_motor.c ../../src/dm/dm_motor_pack.c -o /tmp/test_dm_motor
/tmp/test_dm_motor
