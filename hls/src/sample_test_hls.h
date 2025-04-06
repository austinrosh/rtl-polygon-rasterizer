#ifndef SAMPLE_TEST_HLS
#define SAMPLE_TEST_HLS
#include "rast_types_hls.h"

#pragma hls_design
class SampleTest{
public:
    SampleTest(){}

    #pragma hls_design interface ccore
    bool CCS_BLOCK(run)(TriangleHLS triangle, SampleHLS sample)
    {
        bool isHit;
        // START CODE HERE
        // END CODE HERE
        return isHit;
    }
};

#endif
