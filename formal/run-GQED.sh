#!/bin/bash

# preserve order
cat gqed.sv ../params/rast_params.sv ../rtl/*.sv ./DW_pl_reg.v > rast_gqed.sv
jg -tcl jasper_gqed.tcl -allow_unsupported_OS &
