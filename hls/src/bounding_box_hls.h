#ifndef BOUNDING_BOX_HLS
#define BOUNDING_BOX_HLS

#include "rast_types_hls.h"

#pragma hls_design 
class BoundingBoxGenerator{
public:
    BoundingBoxGenerator() {}

    #pragma hls_design interface
    void CCS_BLOCK(run)(
        ac_channel<TriangleHLS> &triangle_in, 
        ac_channel<ScreenHLS> &screen_in, 
        ac_channel<ConfigHLS> &config_in,
        ac_channel<BoundingBoxHLS> &bbox_out,
        ac_channel<TriangleHLS> &triangle_out,
        ac_channel<ConfigHLS> &config_out
    ){
        #ifndef __SYNTHESIS__
        while(triangle_in.available(1))
        #endif
        {
            TriangleHLS triangle = triangle_in.read();
            ScreenHLS screen = screen_in.read();
            ConfigHLS config = config_in.read();

            BoundingBoxHLS bbox;
            
            // START CODE HERE
            // iterate over remaining vertices
            // round down to subsample grid
            // clip to screen
            // check if bbox is valid
            // write to outputs if bbox is valid
            // END CODE HERE
        }
    }
private:
    SignedFixedPoint min(SignedFixedPoint a, SignedFixedPoint b)
    {
        // START CODE HERE
        // END CODE HERE
    }

    SignedFixedPoint max(SignedFixedPoint a, SignedFixedPoint b)
    {
        // START CODE HERE
        // END CODE HERE
    }

    SignedFixedPoint floor_ss(SignedFixedPoint val, ConfigHLS config)
    {
        // START CODE HERE
        // set lower 8 bits to 0
        // set bits depending on subsample
        // END CODE HERE
        return val;
    }
};

#endif
