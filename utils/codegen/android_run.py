#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# Run a libcamera code generation script from the source tree with the Python
# modules bundled in this Soong host binary (jinja2, yaml and ply), as if it
# had been executed directly.

import os
import runpy
import sys

script = sys.argv[1]
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(script)))
runpy.run_path(script, run_name='__main__')
