# SPDX-License-Identifier: MIT
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

LEAP_SECONDS_SINCE_2004 = int(5)
UNIX_SECONDS_2004 = int(1072915200)

def get_t_its(unix_nanoseconds):
    return int(unix_nanoseconds * 1e-6 + LEAP_SECONDS_SINCE_2004 * 1e3 - UNIX_SECONDS_2004 * 1e3)
