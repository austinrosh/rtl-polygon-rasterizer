#include "../gold/helper.cpp"
#include "../gold/rasterizer.h"

#include <mc_scverify.h>
#include <ac_channel.h>
#include "rast_types_hls.h"

#include "rasterizer_top.cpp"

#define CHECK_VAL(signal, hls, gold) \
    if(hls != gold){ \
        printf("\n[ERROR] Signal %s mismatch!\n", signal); \
        printf("\tHLS: %d\n", hls); \
        printf("\tGold: %d\n", gold); \
        printf("\tError at sample: %d \n", i); \
        CCS_RETURN(-1); \
    }

vector<Sample> hit_samples;

void process_fragment(ZBuff *zbuff, Sample hit_location, Sample subsample, Fragment f){
    hit_samples.push_back(subsample);
}

CCS_MAIN(int argv, char **argc){
    vector<Triangle> triangles;
    Screen screen;
    Config config;
    config.r_shift = 10;

    // Read in test vector file
    char *test_vector_file = getenv("HLS_TEST_VECTOR");
    printf("Using test vector file: %s\n", test_vector_file);
    load_file(test_vector_file, triangles, screen, config);
    
    printf("Found %d triangles\n", triangles.size());
    
    ac_channel<TriangleHLS> triangle_in;
    ac_channel<ScreenHLS> screen_in;
    ac_channel<ConfigHLS> config_in;
    ac_channel<SampleHLS> sample_out;
    ScreenHLS screenHls;
    ConfigHLS configHls;

    screenHls.width = screen.width;
    screenHls.height = screen.height;

    // configHls.ss_w = config.ss_w;
    // configHls.ss_w_lg2 = config.ss_w_lg2;
    
    switch(config.ss){
        case 1:
            configHls.subsample = 8;
            break;
        case 4:
            configHls.subsample = 4;
            break;
        case 16:
            configHls.subsample = 2;
            break;
        case 64:
            configHls.subsample = 1;
            break;
    }
    

    printf("screen.w: %d, screen.h: %d\n", screen.width, screen.height);
    printf("ss: %d, subsample: %d \n", config.ss, configHls.subsample);
    printf("ss_w: %d, ss_w_lg2: %d, ss_i: %f, ss:%d\n", config.ss_w, config.ss_w_lg2, config.ss_i, config.ss);

    for(size_t i = 0; i < triangles.size(); i++){
        TriangleHLS input_triangle;
        for(int vertex = 0; vertex < 3; vertex++){
            input_triangle.v[vertex].x = triangles[i].v[vertex].x;
            input_triangle.v[vertex].y = triangles[i].v[vertex].y;
            input_triangle.v[vertex].z = triangles[i].v[vertex].z;
        }
        triangle_in.write(input_triangle);
        screen_in.write(screenHls);
        config_in.write(configHls);
    }

    printf("Running on reference code...\n");
    ZBuff zbuff;
    for(size_t i = 0; i < triangles.size(); i++){
        rasterize_triangle(triangles[i], &zbuff, screen, config);
    }
    
    printf("Running on HLS code...\n");
    Rasterizer rasterizer;
    rasterizer.run(triangle_in, screen_in, config_in, sample_out);

    
    // Compare outputs
    printf("Checking outputs...\n");
    for(size_t i = 0; i < hit_samples.size(); i++){
        if(!sample_out.available(1)){
            printf("ERROR: not enough samples produced!\n");
            CCS_RETURN(-1);
        }
        
        SampleHLS sample_output = sample_out.read();
        Sample sample;
        
        sample.x = sample_output.x;
        sample.y = sample_output.y;

        Sample hit_location;
        hit_location.x = sample.x >> config.r_shift;
        hit_location.y = sample.y >> config.r_shift;
        
        Sample subsample;
        subsample.x = (sample.x - (hit_location.x << config.r_shift)) / config.ss_i;
        subsample.y = (sample.y - (hit_location.y << config.r_shift)) / config.ss_i;
        
        CHECK_VAL("subsample.x", hit_samples[i].x, subsample.x);
        CHECK_VAL("subsample.y", hit_samples[i].y, subsample.y);
    }
    
    
    printf("\nNo errors found!\n");
    CCS_RETURN(0);
}
