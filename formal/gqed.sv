
// ------------------------------------------------------------------
//   Design Unit:    GQED_rtl
// ------------------------------------------------------------------

module gqed #(
   parameter SIGFIG = 24, // Bits in color and position.
   parameter RADIX = 10, // Fraction bits in color and position
   parameter VERTS = 3, // Maximum Vertices in triangle
   parameter AXIS = 3, // Number of axis foreach vertex 3 is (x,y,z).
   parameter COLORS = 3, // Number of color channels
   parameter BOUND = 4 // Max length of input seqeunce
   )
//BMC Controlled Inputs
  ( 
   input logic clk,
   input logic rst,
   input logic [$clog2(BOUND)-1:0] j, //index of input under verification in cpy1
   input logic [3:0] subSample_RnnnnU, // SubSample_Interval

   input logic signed [SIGFIG-1:0] tri_R10S_in [BOUND-1:0][VERTS-1:0][AXIS-1:0], // Tri Position part of the input sequence 
   input logic unsigned [SIGFIG-1:0] color_R10U_in [BOUND-1:0][COLORS-1:0], //color part of the input sequence
   input logic signed [SIGFIG-1:0] screen_RnnnnS_in [BOUND-1:0][1:0], // Screen Dimensions part of the input seqeuence 
   input logic validTri_R10H_cpy1, //Valid Data for Operation
   input logic validTri_R10H_cpy2 //Valid Data for Operation
  );

// ------------------------------------------------------------------   
   
  //Output Signals from rast cpy1 to be analyzed by G-QED
   logic halt_RnnnnL_cpy1;
   logic signed [SIGFIG-1:0] hit_R18S_cpy1[AXIS-1:0]; // Hit Location
   logic unsigned [SIGFIG-1:0] color_R18U_cpy1[COLORS-1:0]; // Color of Tri
   logic hit_valid_R18H_cpy1;      

  //Output Signals from rast cpy2 to be analyzed by G-QED
   logic halt_RnnnnL_cpy2;
   logic signed [SIGFIG-1:0] hit_R18S_cpy2[AXIS-1:0]; // Hit Location
   logic unsigned [SIGFIG-1:0] color_R18U_cpy2[COLORS-1:0]; // Color of Tri
   logic hit_valid_R18H_cpy2;      


// ------------------------------------------------------------------   
//Signals to create G-QED  



  // Signals to save inputs of interest  
   logic [15:0] in_count_cpy1; // keeps track of the sequence number of the input being fed to cpy1
   logic [1:0] in_count_cpy2; // keeps track of the sequence number of the input being fed to cpy2


  // Signals to save and analyze rast outputs  
   //for cpy1
   logic cpy1_done; // 1'b1 if the j^th outputs is produced
   logic signed [SIGFIG-1:0] save_hit_R18S_cpy1[AXIS-1:0][32]; // saves the value of output subsequence of hit_R18S generated from the j^th input 
   logic unsigned [SIGFIG-1:0] save_color_R18U_cpy1[COLORS-1:0][32]; // saves the value of output subsequence of color_R18U generated from the j^th input
   logic signed [15:0] out_count_cpy1; // keeps track of the sequence number of the output being produced
   logic [4:0] index_cpy1; // pointer to save the next output of the output subsequence generated from the j^th input 
   logic halt_RnnnnL_d5_cpy1; // halt_RnnnnL delayed by 5 cycles
   logic halt_RnnnnL_d6_cpy1; // halt_RnnnnL delayed by 6 cycles

   //for cpy2
   logic cpy2_done; // 1'b1 if the 1st output has been produced
   logic signed [SIGFIG-1:0] save_hit_R18S_cpy2[AXIS-1:0][32]; // saves the value of output subsequence of hit_R18S generated from the 1st input
   logic unsigned [SIGFIG-1:0] save_color_R18U_cpy2[COLORS-1:0][32]; // saves the value of output subsequence of color_R18U generated from the 1st input      
   logic signed [2:0] out_count_cpy2; // keeps track of the sequence number of the output being produced
   logic [4:0] index_cpy2; // pointer to save the next output of the output subsequence generated from the 1st input 
   logic halt_RnnnnL_d5_cpy2; // halt_RnnnnL delayed by 5 cycles
   logic halt_RnnnnL_d6_cpy2; // halt_RnnnnL delayed by 6 cycles

   logic out_match; // 1'b1 if the j^th output from cpy1 and 1st output from cpy2 are equal

// ------------------------------------------------------------------   


  /*
     Store data values for the j^th input of the first copy and the 1st input of the second copy
     hint: To feed rast with a valid input, the input-ready signal and input-valid signal need to be high i.e. the design is ready to accept an input and the input being sent is a valid one.
     Signals to use: validTri_R10H_cpy1, halt_RnnnnL_cpy1, in_count_cpy1, validTri_R10H_cpy2, halt_RnnnnL_cpy2, in_count_cpy2, j 
  */     
   always @(posedge clk)
     begin	
        if (rst) begin
           in_count_cpy1 <= 'b0;
           in_count_cpy2 <= 'b0;           
        end else begin 
 	 if (/*Fill*/)         
           in_count_cpy1 <= in_count_cpy1 + 16'b1;
	 if (/*Fill*/)         
           in_count_cpy2 <= in_count_cpy2 + 2'b1;
       end 
     end

// ------------------------------------------------------------------   

  /*
     Write constraints for input signals. SubSample_RnnnnU is a onehot signal and also needs to be held constant across entire run. j and the input sequence needs to be held constant too. If the accelerator is not ready to take in inputs, the host needs to send bubbles by keeping validTri_R10H low.
     Signals to use: subSample_RnnnnU, j, halt_RnnnnL_cpy1, validTri_R10H_cpy1, halt_RnnnnL_cpy2, validTri_R10H_cpy2
     Parameter to use: BOUND 
  */     

onehot_subSample: assume property (@(posedge clk) /*Fill*/);
stable_subSample: assume property (@(posedge clk) /*Fill*/);
stable_j: assume property (@(posedge clk) /*Fill*/);
legal_j: assume property (@(posedge clk) /*Fill*/); // hint: think of out-of-bound conditions  
stable_in_seq: assume property (@(posedge clk) /*Fill*/);

cpy1_rast_not_rdy: assume property(@(posedge clk) /*Fill*/);
cpy2_rast_not_rdy: assume property(@(posedge clk) /*Fill*/);

// ------------------------------------------------------------------   


cpy1_rast_no_more_inputs: assume property(@(posedge clk) in_count_cpy1>j |-> !validTri_R10H_cpy1);
cpy2_rast_no_more_inputs: assume property(@(posedge clk) in_count_cpy2>0 |-> !validTri_R10H_cpy2);


c_screen_cpy1: assume property(@(posedge clk) ~(|cpy1.bbox.invalidate_R10H)); // We only want to send triangles that fit within the screen
c_screen_cpy2: assume property(@(posedge clk) ~(|cpy2.bbox.invalidate_R10H)); // We only want to send triangles that fit within the screen

// ------------------------------------------------------------------   

  /*
     Instantiate rast cpy1 and cpy2. Some signals are connected for you. You need to connect the rest.     Signals to use: in_count_cpy1, tri_R10S_in, color_R10U_in, validTri_R10H_cpy1, screen_RnnnnS_in,  halt_RnnnnL_cpy1, hit_R18S_cpy1, color_R18U_cpy1, hit_valid_R18H_cpy1, validTri_R10H_cpy2, halt_RnnnnL_cpy2, hit_R18S_cpy2, color_R18U_cpy2, hit_valid_R18H_cpy2, j
  */


rast cpy1(.subSample_RnnnnU(subSample_RnnnnU), .clk(clk), .rst(rst), /*Fill*/);
rast cpy2(.subSample_RnnnnU(subSample_RnnnnU), .clk(clk), .rst(rst), /*Fill*/);

// ------------------------------------------------------------------   


  /*
     Create the halt_RnnnnL_d5 and halt_RnnnnL_d6 signals for each copy.
     Hint: Use the dff modules 
     Signals to use: halt_RnnnnL_cpy1, halt_RnnnnL_cpy2, clk, rst 
  */     
//cpy1
       /*
	Code goes here
       */

//cpy2
       /*
	Code goes here
       */

// ------------------------------------------------------------------   



  /*
     Update out_count for cpy1 and cpy2. out_count increments when an entire output subsequence corresponding to an input has been generated. 
     Signals to use: halt_RnnnnL_d5, halt_RnnnnL_d6
  */      
   always @(posedge clk)
     begin
        if (rst) begin
           out_count_cpy1 <= -1;
           out_count_cpy2 <= -1;           
	end else begin
	   if(/*Fill*/)
              out_count_cpy1 <= out_count_cpy1 + 16'b1;
	   if(/*Fill*/)
              out_count_cpy2 <= out_count_cpy2 + 3'b1;
        end      
     end 

// ------------------------------------------------------------------   



  /*
     Store the j^th output subsequence of cpy1 and 1st output subsequence of cpy2.
     Note an output needs to be valid to be stored. 
     Signals to use: hit_R18S_cpy1, color_R18U_cpy1, index_cpy1, halt_RnnnnL_d5_cpy1, out_count_cpy1, hit_valid_R18H_cpy1, hit_R18S_cpy2, color_R18U_cpy2, index_cpy2, halt_RnnnnL_d5_cpy2, out_count_cpy2, hit_valid_R18H_cpy2, j    
  */     
   always @(posedge clk)
     begin
        if (rst) begin
	   save_hit_R18S_cpy1 <= '{default:0}; 
   	   save_color_R18U_cpy1 <= '{default:0};
	   save_hit_R18S_cpy2 <= '{default:0}; 
   	   save_color_R18U_cpy2 <= '{default:0};
	   index_cpy1 <= 'b0;
	   index_cpy2 <= 'b0;
	   cpy1_done <= 'b0;
	   cpy2_done <= 'b0;
        end else begin
	    if (/*Fill*/) begin //cpy1

		/*
		 Code goes here
		*/

            end else if (out_count_cpy1 == j+1)
           	cpy1_done <= 1'b1;
	    end if (/*Fill*/) begin //cpy2

  		/*
		 Code goes here
		*/

            end else if (out_count_cpy2 == 1)
           	cpy2_done <= 1'b1;
	end
     end 
// ------------------------------------------------------------------   
 
  
 
  /*
     Write conditions to assign out_match.
     Signals to use: save_hit_R18S_cpy1, save_color_R18U_cpy1, save_hit_R18S_cpy2, save_color_R18U_cpy2  
  */     
   assign out_match = /*Fill*/ ;

// ------------------------------------------------------------------   


  /*
     Write the final qed_check property
     Signals to use: cpy1_done, cpy2_done, in_match, out_match
  */     
   assert_functional_consistency : assert property (
       @(posedge clk)
          /*Fill*/  );
// ------------------------------------------------------------------   

removing_trivial_case1: assume property (@(posedge clk) cpy1_done & cpy2_done |-> index_cpy1>0 && index_cpy2>0); //with no valid output always fc 
removing_trivial_case2: assume property (@(posedge clk) j>0); //the runs in the two copies will become identical
         
endmodule


