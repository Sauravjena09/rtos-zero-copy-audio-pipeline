#include "audio_pipeline.h"
#include <arm_math.h>

__ccm_noinit_section static int16_t left_q15[FRAMES_PER_BLOCK] __aligned(4);
__ccm_noinit_section static int16_t right_q15[FRAMES_PER_BLOCK] __aligned(4);
__ccm_noinit_section static float32_t left_f32[FRAMES_PER_BLOCK] __aligned(4);
__ccm_noinit_section static float32_t right_f32[FRAMES_PER_BLOCK] __aligned(4);
__ccm_bss_section static float32_t echoline[4096] __aligned(4);
__ccm_bss_section static float32_t biquad_state_l[4 * 2] __aligned(4);
__ccm_bss_section static float32_t biquad_state_r[4 * 2] __aligned(4);

static const float32_t biquad_cofficient[10] = {
    0.000416f, 0.000833f, 0.000416f, 1.734725f, -0.766006f, 1.0f, 2.0f, 1.0f, 1.852116f, -0.885399f
};

static arm_biquad_casd_df1_inst_f32 biquad_left;
static arm_biquad_casd_df1_inst_f32 biquad_right;

void dsp_filter_init(void)
{
    arm_biquad_cascade_df1_init_f32(&biquad_left, 2U, biquad_cofficient, biquad_state_l);
    arm_biquad_cascade_df1_init_f32(&biquad_right, 2U, biquad_cofficient, biquad_state_r);
}
static uint32_t echo_idx = 0;
void process_audio_block(int16_t *interleaved_dma_buf){
    /*DE-interleave*/
    for(int i=0; i<FRAMES_PER_BLOCK;i++){
      left_q15[i]=interleaved_dma_buf[2*i];
      right_q15[i]=interleaved_dma_buf[2*i+1]; 
    }
    /*type conversion q15 to float32*/
    arm_q15_to_float(left_q15,left_f32,FRAMES_PER_BLOCK);
    arm_q15_to_float(right_q15,right_f32,FRAMES_PER_BLOCK);

    /*biquad processing*/
    arm_biquad_cascade_df1_f32(&biquad_left,left_f32,left_f32,FRAMES_PER_BLOCK);
    arm_biquad_cascade_df1_f32(&biquad_right,right_f32,right_f32,FRAMES_PER_BLOCK);
   
/*CIRCULAR ECHO APPLICATION*/
    for(int i=0;i<FRAMES_PER_BLOCK;i++){
        float32_t delayed=echoline[echo_idx];
        float32_t out=(0.70f * left_f32[i]) +(0.45f*delayed);
        echoline[echo_idx]=out;
        left_f32[i]=out;
        echo_idx=(echo_idx+1)&(4095U);

    }

arm_float_to_q15(left_f32,left_q15,FRAMES_PER_BLOCK);
arm_float_to_q15(right_f32,right_q15,FRAMES_PER_BLOCK);

 /*re interleave step*/
for(int i=0;i<FRAMES_PER_BLOCK;i++){
    interleaved_dma_buf[2*i]=left_q15[i];
    interleaved_dma_buf[2*i+1]=right_q15[i];
}

}