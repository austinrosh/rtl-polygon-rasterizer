#include "jitter_hls.h"
#include "sample_test_hls.h"

class TestIterator{
public:
    TestIterator(){}
    
    #pragma hls_design interface
    void CCS_BLOCK(run)(
        ac_channel<BoundingBoxHLS> &bbox_in, 
        ac_channel<TriangleHLS> &triangle_in,
        ac_channel<ConfigHLS> &config_in,
        ac_channel<SampleHLS> &sample_out   
    ){
        #ifndef __SYNTHESIS__
        while(triangle_in.available(1))
        #endif
        {
            BoundingBoxHLS bbox = bbox_in.read();
            TriangleHLS triangle = triangle_in.read();
            ConfigHLS config = config_in.read();

            // START CODE HERE
            // Create increment value from config.subsample
            // Iterate over box (using normal for loops)
                    // jitter sample
                    // test sample
                    // if hit, write out the sample (including RGB values)
        // END CODE HERE
        }
    }
private:
    SampleTest sampleTest;
    JitterSample jitterSample;
};
