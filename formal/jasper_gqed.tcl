clear -all
analyze -sv09 rast_gqed.sv
elaborate -disable_auto_bbox -top gqed
clock clk
reset -expression rst
prove -bg -property {gqed.assert_functional_consistency}
#prove -bg -property {<embedded>::gqed.assert_functional_consistency}
