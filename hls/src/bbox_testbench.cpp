#include "../gold/helper.cpp"
#include "../gold/rasterizer.h"

#include <mc_scverify.h>
#include <ac_channel.h>
#include "rast_types_hls.h"

#include "rasterizer_top.cpp"

#define CHECK_VAL(signal, gold, hls) \
    if((int)hls != gold){ \
        printf("\n[ERROR] Signal %s mismatch!\n", signal); \
        printf("\tHLS: %d\n", (int)hls); \
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
    load_file("/afs/ir/class/ee271/project/vect/vec_271_04_sv_short.dat", triangles, screen, config);
    
    printf("Found %d triangles\n", triangles.size());
    
    ac_channel<TriangleHLS> triangle_in;
    ac_channel<ScreenHLS> screen_in;
    ac_channel<ConfigHLS> config_in;
    ac_channel<SampleHLS> sample_out;
    ScreenHLS screenHls;
    ConfigHLS configHls;

    screenHls.width = screen.width;
    screenHls.height = screen.height;

    configHls.ss_w = config.ss_w;
    configHls.ss_w_lg2 = config.ss_w_lg2;
    
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
    vector<BoundingBox> gold_bbox_vector;
    for(size_t i = 0; i < triangles.size(); i++){
        BoundingBox gold_bbox = get_bounding_box(triangles[i], screen, config);
        if(gold_bbox.valid){
            gold_bbox_vector.push_back(gold_bbox);
        }
    }

    printf("Running on HLS code...\n");
    BoundingBoxGenerator bbox_generator;
    ac_channel<ConfigHLS> config_out;
    ac_channel<TriangleHLS> triangle_out;
    ac_channel<BoundingBoxHLS> bbox_out;
    bbox_generator.run(triangle_in, screen_in, config_in, bbox_out, triangle_out, config_out);
    
    // Compare outputs
    printf("Checking outputs...\n");
    for(size_t i = 0; i < gold_bbox_vector.size(); i++){

        if(!bbox_out.available(1)){
            printf("ERROR: not enough bbox produced!\n");
            CCS_RETURN(-1);
        }
        
        BoundingBoxHLS bbox_hls = bbox_out.read();
        BoundingBox gold_bbox = gold_bbox_vector[i];
        
        CHECK_VAL("bbox.lower_left.x", gold_bbox.lower_left.x, bbox_hls.lower_left.x);
        CHECK_VAL("bbox.lower_left.y", gold_bbox.lower_left.y, bbox_hls.lower_left.y);
        CHECK_VAL("bbox.upper_right.x", gold_bbox.upper_right.x, bbox_hls.upper_right.x);
        CHECK_VAL("bbox.upper_right.y", gold_bbox.upper_right.y, bbox_hls.upper_right.y);
    }
    
    
    printf("\nNo errors found!\n");
    CCS_RETURN(0);
}
