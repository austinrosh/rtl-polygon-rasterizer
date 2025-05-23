/*
 *  Performs Sample Test on triangle
 *
 *  Inputs:
 *    Sample and triangle Information
 *
 *  Outputs:
 *    Subsample Hit Flag, Subsample location, and triangle Information
 *
 *  Function:
 *    Utilizing Edge Equations determine whether the
 *    sample location lies inside the triangle.
 *    In the simple case of the triangle, this will
 *    occur when the sample lies to one side of all
 *    three lines (either all left or all right).
 *    This corresponds to the minterm 000 and 111.
 *    Additionally, if backface culling is performed,
 *    then only keep the case of all right.
 *
 *  Edge Equation:
 *    For an edge defined as travelling from the
 *    vertice (x_1,y_1) to (x_2,y_2), the sample
 *    (x_s,y_s) lies to the right of the line
 *    if the following expression is true:
 *
 *    0 >  ( x_2 - x_1 ) * ( y_s - y_1 ) - ( x_s - x_1 ) * ( y_2 - y_1 )
 *
 *    otherwise it lies on the line (exactly 0) or
 *    to the left of the line.
 *
 *    This block evaluates the six edges described by the
 *    triangles vertices,  to determine which
 *    side of the lines the sample point lies.  Then it
 *    determines if the sample point lies in the triangle
 *    by or'ing the appropriate minterms.  In the case of
 *    the triangle only three edges are relevant.  In the
 *    case of the quadrilateral five edges are relevant.
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Tue 10/06/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 *
 *
 */

/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxxxN
 * where R indicates that it is a Raster Block signal
 * xxxx indicates the clock slice that it belongs to
 * and N indicates the type of signal that it is.
 * H indicates logic high, L indicates logic low,
 * U indicates unsigned fixed point, and S indicates
 * signed fixed point.
 *
 */

module sampletest
#(
    parameter SIGFIG        = 24, // Bits in color and position.
    parameter RADIX         = 10, // Fraction bits in color and position
    parameter VERTS         = 3, // Maximum Vertices in triangle
    parameter AXIS          = 3, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS        = 3, // Number of color channels
    parameter PIPE_DEPTH    = 2 // How many pipe stages are in this block
)
(
    input logic signed [SIGFIG-1:0]     tri_R16S[VERTS-1:0][AXIS-1:0], // triangle to Iterate Over
    input logic unsigned [SIGFIG-1:0]   color_R16U[COLORS-1:0] , // Color of triangle
    input logic signed [SIGFIG-1:0]     sample_R16S[1:0], // Sample Location
    input logic                         validSamp_R16H, // A valid sample location

    input logic clk, // Clock
    input logic rst, // Reset

    output logic signed [SIGFIG-1:0]    hit_R18S[AXIS-1:0], // Hit Location
    output logic unsigned [SIGFIG-1:0]  color_R18U[COLORS-1:0] , // Color of triangle
    output logic                        hit_valid_R18H                   // Is hit good
);

    localparam EDGES = (VERTS == 3) ? 3 : 5;
    localparam SHORTSF = SIGFIG;
    localparam MROUND = (2 * SHORTSF) - RADIX;

    // output for retiming registers
    logic signed [SIGFIG-1:0]       hit_R18S_retime[AXIS-1:0];   // Hit Location
    logic unsigned [SIGFIG-1:0]     color_R18U_retime[COLORS-1:0];   // Color of triangle
    logic                           hit_valid_R18H_retime;   // Is hit good
    // output for retiming registers

    // Signals in Access Order
    logic signed [SIGFIG-1:0]       tri_shift_R16S[VERTS-1:0][1:0]; // triangle after coordinate shift
    logic signed [SIGFIG-1:0]       edge_R16S[EDGES-1:0][1:0][1:0]; // Edges
    logic signed [(2*SHORTSF)-1:0]  dist_lg_R16S[EDGES-1:0]; // Result of x_1 * y_2 - x_2 * y_1
    logic                           hit_valid_R16H ; // Output (YOUR JOB!)
    logic signed [SIGFIG-1:0]       hit_R16S[AXIS-1:0]; // Sample position
    // Signals in Access Order

    // Your job is to produce the value for hit_valid_R16H signal, which indicates whether a sample lies inside the triangle.
    // hit_valid_R16H is high if validSamp_R16H && sample inside triangle (with back face culling)
    // Consider the following steps:

    // START CODE HERE

    // (1) Shift X, Y coordinates such that the fragment resides on the (0,0) position.
    
    // Declare intermediate variables
    logic signed [SIGFIG-1:0] v0[1:0], v1[1:0], v2[1:0];
    logic cond1, cond2, cond3;

    // Reduced precision
    logic signed [SIGFIG-5:0] v0_trunc[1:0], v1_trunc[1:0], v2_trunc[1:0];
    logic signed [SIGFIG-1:0] trunc_sample_R16S[1:0];

    generate
    for(genvar i = 0; i < 2; i = i + 1) begin
            always_comb begin
                trunc_sample_R16S[i] = {sample_R16S[i][SIGFIG-1:2], 2'd0};
            end
    end
    endgenerate

    // Compute intermediate values
    always_comb begin
        unique case(validSamp_R16H)
        1'b1 : begin
            //Center the sample vertices
            v0[0] = tri_R16S[0][0] - trunc_sample_R16S[0];
            v0[1] = tri_R16S[0][1] - trunc_sample_R16S[1];

            v1[0] = tri_R16S[1][0] - trunc_sample_R16S[0];
            v1[1] = tri_R16S[1][1] - trunc_sample_R16S[1];

            v2[0] = tri_R16S[2][0] - trunc_sample_R16S[0];
            v2[1] = tri_R16S[2][1] - trunc_sample_R16S[1];

            //Reduce precision
            v0_trunc[0] = v0[0][SIGFIG-5:0];
            v0_trunc[1] = v0[1][SIGFIG-5:0];

            v1_trunc[0] = v1[0][SIGFIG-5:0];
            v1_trunc[1] = v1[1][SIGFIG-5:0];

            v2_trunc[0] = v2[0][SIGFIG-5:0];
            v2_trunc[1] = v2[1][SIGFIG-5:0];

            // Multiply logic 
            //cond1 = (v0[0] * v1[1] - v1[0] * v0[1] < 0) || (v0[0] * v1[1] - v1[0] * v0[1] == 0); 
            //cond2 =  v1[0] * v2[1] - v2[0] * v1[1] < 0;
            //cond3 = (v2[0] * v0[1] - v0[0] * v2[1] < 0) || (v2[0] * v0[1] - v0[0] * v2[1] == 0); 
            cond1 = (v0_trunc[0] * v1_trunc[1] - v1_trunc[0] * v0_trunc[1]) <= 0;
            cond2 = (v1_trunc[0] * v2_trunc[1] - v2_trunc[0] * v1_trunc[1]) < 0;
            cond3 = (v2_trunc[0] * v0_trunc[1] - v0_trunc[0] * v2_trunc[1]) <= 0;
            
            // Hit valid calculation
            //hit_valid_R16H = validSamp_R16H && cond1 && cond2 && cond3;
        end
        1'b0 : begin
            v0[0] = 0;
            v0[0] = 0;

            v1[0] = 0;
            v1[1] = 0;

            v2[0] = 0;
            v2[1] = 0;

            v0_trunc[0] = 0;
            v0_trunc[1] = 0;

            v1_trunc[0] = 0;
            v1_trunc[1] = 0;

            v2_trunc[0] = 0;
            v2_trunc[1] = 0;

            cond1 = 0;
            cond2 = 0;
            cond3 = 0;
        end
        endcase
    // Hit valid calculation
         hit_valid_R16H = validSamp_R16H && cond1 && cond2 && cond3;
    end

    // Compute final output
    //assign hit_valid_R16H = validSamp_R16H && cond1 && cond2 && cond3;
    //assert property( @(posedge clk) (dist_lg_R16S[1] == 0) |-> !hit_valid_R16H);

    //Calculate Depth as depth of first vertex
    // Note that a barrycentric interpolation would
    // be more accurate
    always_comb begin
        hit_R16S[1:0] = sample_R16S[1:0]; //Make sure you use unjittered sample
        hit_R16S[2] = tri_R16S[0][2]; // z value equals the z value of the first vertex
    end

    // Flop R16 to R18_retime with retiming registers
    dff2 #(
        .WIDTH          (SIGFIG         ),
        .ARRAY_SIZE     (AXIS           ),
        .PIPE_DEPTH     (PIPE_DEPTH - 1 ),
        .RETIME_STATUS  (1              )
    )
    d_samp_r1
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (hit_R16S       ),
        .out    (hit_R18S_retime)
    );

    dff2 #(
        .WIDTH          (SIGFIG         ),
        .ARRAY_SIZE     (COLORS         ),
        .PIPE_DEPTH     (PIPE_DEPTH - 1 ),
        .RETIME_STATUS  (1              )
    )
    d_samp_r2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (color_R16U         ),
        .out    (color_R18U_retime  )
    );

    dff_retime #(
        .WIDTH          (1              ),
        .PIPE_DEPTH     (PIPE_DEPTH - 1 ),
        .RETIME_STATUS  (1              ) // RETIME
    )
    d_samp_r3
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (1'b1                   ),
        .in     (hit_valid_R16H         ),
        .out    (hit_valid_R18H_retime  )
    );
    /* Flop R16 to R18_retime with retiming registers*/

    /* Flop R18_retime to R18 with fixed registers */
    dff2 #(
        .WIDTH          (SIGFIG ),
        .ARRAY_SIZE     (AXIS   ),
        .PIPE_DEPTH     (1      ),
        .RETIME_STATUS  (0      )
    )
    d_samp_f1
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (hit_R18S_retime),
        .out    (hit_R18S       )
    );

    dff2 #(
        .WIDTH          (SIGFIG ),
        .ARRAY_SIZE     (COLORS ),
        .PIPE_DEPTH     (1      ),
        .RETIME_STATUS  (0      )
    )
    d_samp_f2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (color_R18U_retime  ),
        .out    (color_R18U         )
    );

    dff #(
        .WIDTH          (1  ),
        .PIPE_DEPTH     (1  ),
        .RETIME_STATUS  (0  ) // No retime
    )
    d_samp_f3
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (1'b1                   ),
        .in     (hit_valid_R18H_retime  ),
        .out    (hit_valid_R18H         )
    );

    /* Flop R18_retime to R18 with fixed registers */

endmodule



