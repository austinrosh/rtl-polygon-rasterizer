/*
 * Bounding Box Module
 *
 * Inputs:
 *   3 x,y,z vertices corresponding to tri
 *   1 valid bit, indicating triangle is valid data
 *
 *  Config Inputs:
 *   2 x,y vertices indicating screen dimensions
 *   1 integer representing square root of SS (16MSAA->4)
 *      we will assume config values are held in some
 *      register and are valid given a valid triangle
 *
 *  Control Input:
 *   1 halt signal indicating that no work should be done
 *
 * Outputs:
 *   2 vertices describing a clamped bounding box
 *   1 Valid signal indicating that bounding
 *           box and triangle value is valid
 *   3 x,y vertices corresponding to tri
 *
 * Global Signals:
 *   clk, rst
 *
 * Function:
 *   Determine a bounding box for the triangle
 *   represented by the vertices.
 *
 *   Clamp the Bounding Box to the subsample pixel
 *   space
 *
 *   Clip the Bounding Box to Screen Space
 *
 *   Halt operating but retain values if next stage is busy
 *
 *
 * Long Description:
 *   This bounding box block accepts a triangle described with three
 *   vertices and determines a set of sample points to test against
 *   the triangle.  These sample points correspond to the
 *   either the pixels in the final image or the pixel fragments
 *   that compose the pixel if multisample anti-aliasing (MSAA)
 *   is enabled.
 *
 *   The inputs to the box are clocked with a bank of dflops.
 *
 *   After the data is clocked, a bounding box is determined
 *   for the triangle. A bounding box can be determined
 *   through calculating the maxima and minima for x and y to
 *   generate a lower left vertice and upper right
 *   vertice.  This data is then clocked.
 *
 *   The bounding box next needs to be clamped to the fragment grid.
 *   This can be accomplished through rounding the bounding box values
 *   to the fragment grid.  Additionally, any sample points that exist
 *   outside of screen space should be rejected.  So the bounding box
 *   can be clipped to the visible screen space.  This clipping is done
 *   using the screen signal.
 *
 *   The Halt signal is utilized to hold the current triangle bounding box.
 *   This is because one bounding box operation could correspond to
 *   multiple sample test operations later in the pipe.  As these samples
 *   can take a number of cycles to complete, the data held in the bounding
 *   box stage needs to be preserved.  The halt signal is also required for
 *   when the write device is full/busy.
 *
 *   The valid signal is utilized to indicate whether or not a triangle
 *   is actual data.  This can be useful if the device being read from,
 *   has no more triangles.
 *
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Fri 09/30/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 */


/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxxxN
 * where R indicates that it is a Raster Block signal
 * xxxx indicates the clock slice that it belongs to
 * N indicates the type of signal that it is.
 *    H indicates logic high,
 *    L indicates logic low,
 *    U indicates unsigned fixed point,
 *    S indicates signed fixed point.
 *
 * For all the signed fixed point signals (logic signed [SIGFIG-1:0]),
 * their highest `$sig_fig-$radix` bits, namely [`$sig_fig-1`:RADIX]
 * represent the integer part of the fixed point number,
 * while the lowest RADIX bits, namely [`$radix-1`:0]
 * represent the fractional part of the fixed point number.
 *
 *
 *
 * For signal subSample_RnnnnU (logic [3:0])
 * 1000 for  1x MSAA eq to 1 sample per pixel
 * 0100 for  4x MSAA eq to 4 samples per pixel,
 *              a sample is half a pixel on a side
 * 0010 for 16x MSAA eq to 16 sample per pixel,
 *              a sample is a quarter pixel on a side.
 * 0001 for 64x MSAA eq to 64 samples per pixel,
 *              a sample is an eighth of a pixel on a side.
 *
 */

module bbox
#(
    parameter SIGFIG        = 24, // Bits in color and position.
    parameter RADIX         = 10, // Fraction bits in color and position
    parameter VERTS         = 3, // Maximum Vertices in triangle
    parameter AXIS          = 3, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS        = 3, // Number of color channels
    parameter PIPE_DEPTH    = 3 // How many pipe stages are in this block
)
(
    //Input Signals
    input logic signed [SIGFIG-1:0]     tri_R10S[VERTS-1:0][AXIS-1:0] , // Sets X,Y Fixed Point Values
    input logic unsigned [SIGFIG-1:0]   color_R10U[COLORS-1:0] , // Color of Tri
    input logic                             validTri_R10H , // Valid Data for Operation

    //Control Signals
    input logic                         halt_RnnnnL , // Indicates No Work Should Be Done
    input logic signed [SIGFIG-1:0] screen_RnnnnS[1:0] , // Screen Dimensions
    input logic [3:0]                   subSample_RnnnnU , // SubSample_Interval

    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    //Outout Signals
    output logic signed [SIGFIG-1:0]    tri_R13S[VERTS-1:0][AXIS-1:0], // 4 Sets X,Y Fixed Point Values
    output logic unsigned [SIGFIG-1:0]  color_R13U[COLORS-1:0] , // Color of Tri
    output logic signed [SIGFIG-1:0]    box_R13S[1:0][1:0], // 2 Sets X,Y Fixed Point Values
    output logic                            validTri_R13H,                  // Valid Data for Operation
    output logic                            valid_culled_R13H               // Valid and culled 
    //output logic                            halt_bubble
);

    localparam [RADIX-1:0] MASK1 = { RADIX{1'b0} };
    localparam [RADIX-1:0] MASK2 = { 1'b1, {(RADIX-1){1'b0}} };
    localparam [RADIX-1:0] MASK3 = { 2'b11, {(RADIX-2){1'b0}} }; 
    localparam [RADIX-1:0] MASK4 = { 3'b111, {(RADIX-3){1'b0}} };

    //Signals In Clocking Order

    //Begin R10 Signals

    // Step 1 Result: LL and UR X, Y Fixed Point Values determined by calculating min/max vertices
    // box_R10S[0][0]: LL X
    // box_R10S[0][1]: LL Y
    // box_R10S[1][0]: UR X
    // box_R10S[1][1]: UR Y
    logic signed [SIGFIG-1:0]   box_R10S[1:0][1:0];
    // Step 2 Result: LL and UR Rounded Down to SubSample Interval
    logic signed [SIGFIG-1:0]   rounded_box_R10S[1:0][1:0];
    // Step 3 Result: LL and UR X, Y Fixed Point Values after Clipping
    logic signed [SIGFIG-1:0]   out_box_R10S[1:0][1:0];      // bounds for output
    // Step 3 Result: valid if validTri_R10H && BBox within screen
    logic   outvalid_R10H;               // output is valid
    logic   valid_culled_R10H;           // output valid and culled

    //End R10 Signals

    // Begin output for retiming registers
    logic signed [SIGFIG-1:0]   tri_R13S_retime[VERTS-1:0][AXIS-1:0]; // 4 Sets X,Y Fixed Point Values
    logic unsigned [SIGFIG-1:0] color_R13U_retime[COLORS-1:0];        // Color of Tri
    logic signed [SIGFIG-1:0]   box_R13S_retime[1:0][1:0];             // 2 Sets X,Y Fixed Point Values
    logic                           validTri_R13H_retime ;                 // Valid Data for Operation
    logic                           valid_culled_R13H_retime;              // Valid culled 
    // End output for retiming registers

    // ********** Step 1:  Determining a Bounding Box **********
    // Here you need to determine the bounding box by comparing the vertices
    // and assigning box_R10S to be the proper coordinates

    // START CODE HERE
    
    // Bounding Box logic 
    logic v0_x_min, v0_y_min, v0_x_max, v0_y_max; 
    always_comb begin
        
        v0_x_min = (tri_R10S[0][0] < tri_R10S[1][0]) && (tri_R10S[0][0] < tri_R10S[2][0]); 
        v0_y_min = (tri_R10S[0][1] < tri_R10S[1][1]) && (tri_R10S[0][1] < tri_R10S[2][1]); 
        v0_x_max = (tri_R10S[0][0] > tri_R10S[1][0]) && (tri_R10S[0][0] > tri_R10S[2][0]); 
        v0_y_max = (tri_R10S[0][1] > tri_R10S[1][1]) && (tri_R10S[0][1] > tri_R10S[2][1]); 
        
        box_R10S[0][0] = v0_x_min ? tri_R10S[0][0] : (tri_R10S[1][0] < tri_R10S[2][0]) ? tri_R10S[1][0] : tri_R10S[2][0];
        box_R10S[0][1] = v0_y_min ? tri_R10S[0][1] : (tri_R10S[1][1] < tri_R10S[2][1]) ? tri_R10S[1][1] : tri_R10S[2][1];
        box_R10S[1][0] = v0_x_max ? tri_R10S[0][0] : (tri_R10S[1][0] > tri_R10S[2][0]) ? tri_R10S[1][0] : tri_R10S[2][0];
        box_R10S[1][1] = v0_y_max ? tri_R10S[0][1] : (tri_R10S[1][1] > tri_R10S[2][1]) ? tri_R10S[1][1] : tri_R10S[2][1];
        
    end
    
    // END CODE HERE

    // Assertions to check if box_R10S is assigned properly
    // 1. Each of the coordinates box_R10S are always and uniquely assigned
    // 2. Upper right coordinate is never less than lower left

    //assert that lower left is less than upper right for x & y coordinates
    assert property(@(posedge clk) box_R10S[0][0] <= box_R10S[1][0]) else $fatal("LL/UR x bounds failed");
    assert property(@(posedge clk) box_R10S[0][1] <= box_R10S[1][1]) else $fatal("LL/UR y bounds failed");

    // ***************** End of Step 1 *********************


    // ********** Step 2:  Round Values to Subsample Interval **********

    // We will use the floor operation for rounding.
    // To floor a signal, we simply turn all of the bits
    // below a specific RADIX to 0.
    // The complication here is that there are 4 setting.
    // 1x MSAA eq. to 1 sample per pixel
    // 4x MSAA eq to 4 samples per pixel, a sample is
    // half a pixel on a side
    // 16x MSAA eq to 16 sample per pixel, a sample is
    // a quarter pixel on a side.
    // 64x MSAA eq to 64 samples per pixel, a sample is
    // an eighth of a pixel on a side.


    // Handle bit mask from MSAA signal
    logic [RADIX-1:0] bit_mask;

    always_comb begin
        unique case (subSample_RnnnnU)
            4'b1000: bit_mask = MASK1;  // Keep no decimal bits 
            4'b0100: bit_mask = MASK2;  // Keep only first decimal bit
            4'b0010: bit_mask = MASK3;  // Keep only first 2 decimal bits
            4'b0001: bit_mask = MASK4;  // Keep only first 3 decimal bits
        endcase
    end

    //Round LowerLeft and UpperRight for X and Y
    generate
    for(genvar i = 0; i < 2; i = i + 1) begin
        for(genvar j = 0; j < 2; j = j + 1) begin
            always_comb begin
                rounded_box_R10S[i][j][SIGFIG-1:RADIX] = box_R10S[i][j][SIGFIG-1:RADIX];
                rounded_box_R10S[i][j][RADIX-1:0] = box_R10S[i][j][RADIX-1:0] & bit_mask;
            end
        end
    end
    endgenerate

    //Assertion to help you debug errors in rounding
    assert property( @(posedge clk) (box_R10S[0][0] - rounded_box_R10S[0][0]) <= {subSample_RnnnnU,7'b0});
    assert property( @(posedge clk) (box_R10S[0][1] - rounded_box_R10S[0][1]) <= {subSample_RnnnnU,7'b0});
    assert property( @(posedge clk) (box_R10S[1][0] - rounded_box_R10S[1][0]) <= {subSample_RnnnnU,7'b0});
    assert property( @(posedge clk) (box_R10S[1][1] - rounded_box_R10S[1][1]) <= {subSample_RnnnnU,7'b0});

    // ***************** End of Step 2 *********************


    // ***************** Intermediate Step: Backface Culling ********
    /*
    logic right_dir; 
    logic signed [SIGFIG-1:0]               trunc_bfc_R10S[VERTS-1:0][AXIS-1:0];

    generate
        for(genvar i = 0; i < VERTS; i = i + 1) begin
            for(genvar j = 0; j < AXIS; j = j + 1) begin
                always_comb begin
                    trunc_bfc_R10S[i][j] = {tri_R10S[i][j][SIGFIG-1:2], 2'd0};
                end
            end
        end
    endgenerate
    

    always_comb begin
     right_dir =   (trunc_bfc_R10S[1][0] - trunc_bfc_R10S[0][0]) * (trunc_bfc_R10S[2][1] - trunc_bfc_R10S[0][1]) -  
                   (trunc_bfc_R10S[1][1] - trunc_bfc_R10S[0][1]) * (trunc_bfc_R10S[2][0] - trunc_bfc_R10S[0][0]) < 0; 
    end
    */

    // Backface Culling
    logic                      back_facing; 
    logic signed [SIGFIG-1:0]  edge_dx_0, edge_dx_1; // Edge x-differences
    logic signed [SIGFIG-1:0]  edge_dy_0, edge_dy_1; // Edge y-differences
    logic signed [12:0]        truncated_dx_0, truncated_dx_1; // Truncated x-differences
    logic signed [12:0]        truncated_dy_0, truncated_dy_1; // Truncated y-differences
    logic signed [24:0]        determinant; // Determinant of the triangle

    // Calculate x and y differences and truncate to lower precision
    always_comb begin
        edge_dx_0 = tri_R10S[1][0] - tri_R10S[0][0];
        edge_dx_1 = tri_R10S[2][0] - tri_R10S[1][0];
        truncated_dx_0 = edge_dx_0[12:0];
        truncated_dx_1 = edge_dx_1[12:0];

        edge_dy_0 = tri_R10S[2][1] - tri_R10S[1][1];
        edge_dy_1 = tri_R10S[1][1] - tri_R10S[0][1];
        truncated_dy_0 = edge_dy_0[12:0];
        truncated_dy_1 = edge_dy_1[12:0];

        // Compute 2D determinant to get triangle's normal vector direction
        determinant = (truncated_dx_0 * truncated_dy_0) - (truncated_dx_1 * truncated_dy_1);
        back_facing = (determinant > 0) ? 1'b0 : validTri_R10H;
    end
  
    
    // ***************** End of Backface Culling *********************


    // ********** Step 3:  Clipping or Rejection **********

    // Clamp if LL is down/left of screen origin
    // Clamp if UR is up/right of Screen
    // Invalid if BBox is up/right of Screen
    // Invalid if BBox is down/left of Screen
    // outvalid_R10H high if validTri_R10H && BBox is valid

    always_comb begin
        if (halt_RnnnnL) begin
    
        // START CODE HERE

            // Clamp LL 
            out_box_R10S[0][0] = (rounded_box_R10S[0][0] < 0) ? 0 : rounded_box_R10S[0][0];
            out_box_R10S[0][1] = (rounded_box_R10S[0][1] < 0) ? 0 : rounded_box_R10S[0][1];

            // Clamp UR 
            out_box_R10S[1][0] = (rounded_box_R10S[1][0] > screen_RnnnnS[0]) ? screen_RnnnnS[0] : rounded_box_R10S[1][0];
            out_box_R10S[1][1] = (rounded_box_R10S[1][1] > screen_RnnnnS[1]) ? screen_RnnnnS[1] : rounded_box_R10S[1][1];

            // Valid output signal (not actual)
            outvalid_R10H = validTri_R10H && (out_box_R10S[0][0] <= out_box_R10S[1][0]) && (out_box_R10S[0][1] <= out_box_R10S[1][1]);
            

            // Valid output signal (actual valid signal)
            valid_culled_R10H = outvalid_R10H && back_facing; 
        //halt_bubble = halt_RnnnnL || (~valid_culled_R13H && validTri_R13H); 
            
            // END CODE HERE
        end

    end

    
    //Assertion for checking if outvalid_R10H has been assigned properly
    assert property( @(posedge clk) (outvalid_R10H |-> out_box_R10S[1][0] <= screen_RnnnnS[0] ));
    assert property( @(posedge clk) (outvalid_R10H |-> out_box_R10S[1][1] <= screen_RnnnnS[1] ));

    // ***************** End of Step 3 *********************

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_bbx_r1
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (tri_R10S          ),
        .out    (tri_R13S_retime   )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_bbx_r2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (color_R10U         ),
        .out    (color_R13U_retime  )
    );

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_bbx_r3
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (halt_RnnnnL    ),
        .in     (out_box_R10S   ),
        .out    (box_R13S_retime)
    );

    dff_retime #(
        .WIDTH(1),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1) // Retime
    )
    d_bbx_r4
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (halt_RnnnnL           ),
        .in     (outvalid_R10H          ),
        .out    (validTri_R13H_retime   )
    );

    dff_retime #(
        .WIDTH(1),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1) // Retime
    )
    d_bbx_r5
    (
        .clk    (clk                        ),
        .reset  (rst                        ),
        .en     (halt_RnnnnL                ),
        .in     (valid_culled_R10H          ),
        .out    (valid_culled_R13H_retime   )
    );


    //Flop Clamped Box to R13_retime with retiming registers

    //Flop R13_retime to R13 with fixed registers
    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_bbx_f1
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (tri_R13S_retime    ),
        .out    (tri_R13S           )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_bbx_f2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (color_R13U_retime  ),
        .out    (color_R13U         )
    );

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_bbx_f3
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (halt_RnnnnL    ),
        .in     (box_R13S_retime),
        .out    (box_R13S       )
    );

    dff #(
        .WIDTH(1),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d_bbx_f4
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (halt_RnnnnL           ),
        .in     (validTri_R13H_retime   ),
        .out    (validTri_R13H          )
    );

    dff #(  
        .WIDTH(1),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d_bbx_f5
    (
        .clk    (clk                        ),
        .reset  (rst                        ),
        .en     (halt_RnnnnL               ),
        .in     (valid_culled_R13H_retime   ),
        .out    (valid_culled_R13H          )
    );
    
    
    //Flop R13_retime to R13 with fixed registers

    //Error Checking Assertions

    //Define a Less Than Property
    //
    //  a should be less than b
    property rb_lt( rst, a, b, c );
        @(posedge clk) rst | ((a<=b) | !c);
    endproperty

    //Check that Lower Left of Bounding Box is less than equal Upper Right
    assert property( rb_lt( rst, box_R13S[0][0], box_R13S[1][0], validTri_R13H ));
    assert property( rb_lt( rst, box_R13S[0][1], box_R13S[1][1], validTri_R13H ));
    //Check that Lower Left of Bounding Box is less than equal Upper Right

    //Error Checking Assertions

endmodule








