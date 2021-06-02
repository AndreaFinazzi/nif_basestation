#!/usr/bin/env bash
docker run -it -v $(pwd):/workspace --network="host" -w /workspace bst_viz