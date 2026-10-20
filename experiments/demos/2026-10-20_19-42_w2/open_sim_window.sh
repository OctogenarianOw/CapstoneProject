#!/bin/bash
# Open a Gazebo GUI client on the headless server PX4 started.
# The server stays headless (spec: HEADLESS=1, WSL2 dxgkrnl bug); this
# separate client renders in software (llvmpipe) and may be slow, but if
# it fails nothing else is affected. A wrapper so that the session
# command line holds no cleanup pattern.
exec gz sim -g
