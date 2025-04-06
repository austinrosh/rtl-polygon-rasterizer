#ifndef JITTER_HLS
#define JITTER_HLS

#include "rast_types_hls.h"

#pragma hls_design
class Hash40to8{
public:
  Hash40to8(){}

  #pragma hls_design interface ccore
  ac_int<8, false> CCS_BLOCK(run)(ac_int<40, false> input, ac_int<8, false> mask){
    ac_int<32, false> arr32;

    ac_int<8,false> input_0 = input.slc<8>(0);
    ac_int<8,false> input_1 = input.slc<8>(16);
    ac_int<8,false> input_2 = input.slc<8>(24);
    ac_int<8,false> input_3 = input.slc<8>(32);

    arr32.set_slc(0, input.slc<8>(0) ^ input.slc<8>(8));
    arr32.set_slc(8, input.slc<8>(8) ^ input.slc<8>(16));
    arr32.set_slc(16, input.slc<8>(16) ^ input.slc<8>(24));
    arr32.set_slc(24, input.slc<8>(24) ^ input.slc<8>(32));

    ac_int<16, false> arr16;
    arr16.set_slc(0, arr32.slc<8>(0) ^ arr32.slc<8>(16));
    arr16.set_slc(8, arr32.slc<8>(8) ^ arr32.slc<8>(24));
    
    ac_int<8,false> out1 = arr16.slc<8>(0);
    ac_int<8,false> out2 = arr16.slc<8>(8);

    ac_int<8, false> output= (arr16.slc<8>(0) ^ arr16.slc<8>(8));
    output = output & mask;

    return output;
  }
};

#pragma hls_design
class JitterSample{
public:
  JitterSample(){}

  #pragma hls_design interface ccore
  SampleHLS CCS_BLOCK(run)(SampleHLS sample, ConfigHLS config){
    ac_int<8, false> mask = 0;
    
    switch(config.subsample){
      case 1:
        mask.set_slc(0, (ac_int<5,false>)0xFF);
        break;
      case 2:
        mask.set_slc(0, (ac_int<6,false>)0xFF);
        break;
      case 4:
        mask.set_slc(0, (ac_int<7,false>)0xFF);  
        break;
      case 8:
        mask.set_slc(0, (ac_int<8,false>)0xFF);
        break;
    }
    
    ac_int<40, false> x_hash_in;
    ac_int<40, false> y_hash_in;
    
    x_hash_in.set_slc(0, sample.x.slc<20>(4));
    x_hash_in.set_slc(20, sample.y.slc<20>(4));

    y_hash_in.set_slc(0, sample.y.slc<20>(4));
    y_hash_in.set_slc(20, sample.x.slc<20>(4));

    ac_int<8, false> x_hash_out;
    ac_int<8, false> y_hash_out;

    x_hash_out = x_hash.run(x_hash_in, mask);
    y_hash_out = y_hash.run(y_hash_in, mask);
    
    SampleHLS jitteredSample;
    jitteredSample.x = 0;
    jitteredSample.x.set_slc(2, x_hash_out);
    jitteredSample.x = sample.x | jitteredSample.x;
    
    jitteredSample.y = 0;
    jitteredSample.y.set_slc(2, y_hash_out);
    jitteredSample.y = sample.y | jitteredSample.y;

    return jitteredSample;
  }

private:
  Hash40to8 x_hash;
  Hash40to8 y_hash;
};

#endif
