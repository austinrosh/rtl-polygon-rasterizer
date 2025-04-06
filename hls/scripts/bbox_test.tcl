project new -name bbox_test
project save

options set Input/TargetPlatform x86_64
options set Output/OutputVHDL false

flow package require SCVerify
flow package option set /SCVerify/USE_CCS_BLOCK true
flow package option set /SCVerify/USE_NCSIM true
flow package option set /SCVerify/USE_VCS false
flow package option set /SCVerify/USE_MSIM false

flow package require /NCSim
solution options set Flows/NCSim/NC_ROOT /cad/cadence/INCISIVE15.20.022/

solution file add src/rasterizer_top.cpp
solution file add src/bbox_testbench.cpp -exclude true
solution file add ../gold/rasterizer.c -exclude true

go analyze

directive set -DESIGN_HIERARCHY {
    {BoundingBoxGenerator}
}

solution library add nangate-45nm_beh -- -rtlsyntool DesignCompiler -vendor Nangate -technology 045nm

go libraries