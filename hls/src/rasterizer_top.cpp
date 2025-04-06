#include <ac_channel.h>
#include "rast_types_hls.h"

#include <mc_scverify.h>

#include "bounding_box_hls.h"
#include "jitter_hls.h"
#include "sample_test_hls.h"
#include "test_iterator_hls.h"

#pragma hls_design top
class Rasterizer{
public:
    Rasterizer(){}

    #pragma hls_design interface
    void CCS_BLOCK(run)(
        ac_channel<TriangleHLS> &triangle_in, 
        ac_channel<ScreenHLS> &screen_in, 
        ac_channel<ConfigHLS> &config_in, 
        ac_channel<SampleHLS> &sample_out
    ){
        boundingBox.run(
            // Inputs
            triangle_in,
            screen_in,
            config_in,
            // Outputs
            boundingbox_bbox2testit,
            triangle_bbox2testit,
            config_bbox2testit
        );

        testIterator.run(
            // Inputs
            boundingbox_bbox2testit,
            triangle_bbox2testit,
            config_bbox2testit,
            // Outputs
            sample_out
        );
    }

private:
    BoundingBoxGenerator boundingBox;
    TestIterator testIterator;

    // Bounding box to Test Iterator
    ac_channel<BoundingBoxHLS> boundingbox_bbox2testit;
    ac_channel<TriangleHLS> triangle_bbox2testit;
    ac_channel<ConfigHLS> config_bbox2testit;

    // // Test Iterator to Sample Test
    // ac_channel<TriangleHLS> triangle_testit2stest;
    // ac_channel<SampleHLS> sample_testit2stest;
    // ac_channel<ConfigHLS> config_testit2stest;
};
