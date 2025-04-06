set CLK_PERIOD $::env(CLOCK_PERIOD)
set CLK_HIGH [expr $CLK_PERIOD/2]
set name clk

project new -name build
project save

options set Input/TargetPlatform x86_64
options set Output/OutputVHDL false
options set Flows/DesignCompiler/MapEffort high
options set Flows/DesignCompiler/OutNetlistFormat verilog
options set Flows/DesignCompiler/EnableClockGating true
options set ComponentLibs/TechLibSearchPath ../lib
flow package require SCVerify
flow package option set /SCVerify/USE_CCS_BLOCK true
flow package option set /SCVerify/USE_NCSIM true
flow package option set /SCVerify/USE_VCS false
flow package option set /SCVerify/USE_MSIM false

flow package require /NCSim
solution options set Flows/NCSim/NC_ROOT /cad/cadence/INCISIVE15.20.022/

solution file add src/rasterizer_top.cpp
solution file add src/rasterizer_testbench.cpp -exclude true
solution file add ../gold/rasterizer.c -exclude true

go analyze

solution library add nangate-45nm_beh -- -rtlsyntool DesignCompiler -vendor Nangate -technology 045nm

go libraries

directive set -CLOCKS "$name \"-CLOCK_PERIOD $CLK_PERIOD -CLOCK_EDGE rising -CLOCK_UNCERTAINTY 0.0 -CLOCK_HIGH_TIME $CLK_HIGH -RESET_SYNC_NAME rst -RESET_ASYNC_NAME arst_n -RESET_KIND sync -RESET_SYNC_ACTIVE high -RESET_ASYNC_ACTIVE low -ENABLE_NAME {} -ENABLE_ACTIVE high\"    "

directive set /Rasterizer -INPUT_DELAY [expr $CLK_PERIOD*3/4]
directive set /Rasterizer -OUTPUT_DELAY [expr $CLK_PERIOD*3/4]

go assembly

go architect

go extract
