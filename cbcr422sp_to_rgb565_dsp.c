/**
 * \file  cbcr422sp_to_rgb565_dsp.c
 * 
 * \brief Colorspace conversion from YCbCr422 semi-planar to RGB565 planar.
 *        This function is to be used by the DSP core.
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef _TMS320C6X
#pragma CODE_SECTION(cbcr422sp_to_rgb565_c, ".text:intrinsic");

// Constants
unsigned p1_color = 0x001F; // Blue
unsigned p2_color = 0xA000; // Dark Red
unsigned skin_color = 0xFFFF;
int min_thresh = 2;
int max_thresh = 21;
unsigned color;

void cbcr422sp_to_rgb565_c
(
    const unsigned char *restrict cbcr_src,  /* Blue color-difference (B'-Y') */
    unsigned int num_lines,                  /* Height (480) */
    unsigned int src_pitch,
    const short coeff[5],                    /* Matrix coefficients. */
    const unsigned char *restrict y_data,    /* Luminence data (Y') */
    unsigned int y_pitch,
    unsigned short *restrict rgb_data,       /* RGB 5:6:5 packed pixel output. */
    unsigned int rgb_pitch,
    unsigned num_pixels,                     /* Width (640) and # of luma pixels to process. */
    unsigned char *player1,
    unsigned char *player2,
    unsigned char *RPS_disp_p1,
    unsigned char *RPS_disp_p2,
    unsigned char *score_disp_p1,
    unsigned char *score_disp_p2
)
{
    unsigned int i, j;
    unsigned char *restrict cbcr_src_p;
    unsigned char *restrict y_data_p;
    unsigned short *restrict rgb_data_p;

    unsigned player_width = num_pixels / 2;

    long long cr3_cb3_cr2_cb2_cr1_cb1_cr0_cb0;
    int cr3_cb3_cr2_cb2, cr1_cb1_cr0_cb0;

    int cb3cb2cb1cb0, cr3cr2cr1cr0;

    const unsigned c0   = _pack2(coeff[0], coeff[0]);
    const unsigned c1   = _pack2(coeff[1], coeff[1]);
    const unsigned c2c3 = _pack2(coeff[2], coeff[3]);
    const unsigned c3c2 = _pack2(coeff[3], coeff[2]);
    const unsigned c4   = _pack2(coeff[4], coeff[4]);

    _nassert((int)y_data   % 8 == 0);
    _nassert((int)cbcr_src % 8 == 0);
    _nassert((int)rgb_data % 8 == 0);
    
    /* ---------------------------------------------------------------- */
    /* For each line.                                                   */
    /* ---------------------------------------------------------------- */    
     for (i = 0; i < num_lines; i++) 
     {
        /* ---------------------------------------------------------------- */
        /* Get the right pointers.                                          */
        /* ---------------------------------------------------------------- */
        cbcr_src_p = (unsigned char *)cbcr_src + i * src_pitch;
        y_data_p = (unsigned char *)y_data + i * y_pitch;
        rgb_data_p = (unsigned short *)rgb_data + i * rgb_pitch;
        
        /* ---------------------------------------------------------------- */
        /* For each set of 8 pixels (8 Y value, 4 Cb, and 4 Cr).            */
        /* ---------------------------------------------------------------- */
        for (j = 0; j < num_pixels; j+=8)
        {
            unsigned y_7654_ = _amem4_const(&y_data_p [j + 4]);
            unsigned y_3210_ = _amem4_const(&y_data_p [j + 0]);

            /* ---------------------------------------------------------------- */
            /*  Separate the Cb and Cr value.                                   */
            /* ---------------------------------------------------------------- */
            cr3_cb3_cr2_cb2_cr1_cb1_cr0_cb0 = _amem8 (cbcr_src_p);
            cbcr_src_p += 8;

            cr3_cb3_cr2_cb2 = _hill (cr3_cb3_cr2_cb2_cr1_cb1_cr0_cb0);
            cr1_cb1_cr0_cb0 = _loll (cr3_cb3_cr2_cb2_cr1_cb1_cr0_cb0);
    
            cb3cb2cb1cb0    = _packl4(cr3_cb3_cr2_cb2, cr1_cb1_cr0_cb0);
            cr3cr2cr1cr0    = _packh4(cr3_cb3_cr2_cb2, cr1_cb1_cr0_cb0);
    
            unsigned cb6420_ = cb3cb2cb1cb0 ^ 0x80808080;
            unsigned cr6420_ = cr3cr2cr1cr0 ^ 0x80808080;

            /* ---------------------------------------------------------------- */
            /*  Unpack the chroma and left-shift it by 3.                       */
            /* ---------------------------------------------------------------- */
            double   cb6420  = _mpysu4(cb6420_, 0x08080808);
            double   cr6420  = _mpysu4(cr6420_, 0x08080808);
            unsigned cb64    = _hi(cb6420);
            unsigned cb20    = _lo(cb6420);
            unsigned cr64    = _hi(cr6420);
            unsigned cr20    = _lo(cr6420);

            /* ---------------------------------------------------------------- */
            /*  Unpack the luma, left-shift it by 3, and remove the +16 bias.   */
            /* ---------------------------------------------------------------- */
            double   y_7654  = _mpyu4(   y_7654_, 0x08080808);
            double   y_3210  = _mpyu4(   y_3210_, 0x08080808);
            unsigned y_76    = _sub2(_hi(y_7654), 0x00800080);
            unsigned y_54    = _sub2(_lo(y_7654), 0x00800080);
            unsigned y_32    = _sub2(_hi(y_3210), 0x00800080);
            unsigned y_10    = _sub2(_lo(y_3210), 0x00800080);

            /* ---------------------------------------------------------------- */
            /*  Perform leftmost column of matrix multiply.  We generate 8      */
            /*  separate 16Q16 products for the 8 output pixels.                */
            /* ---------------------------------------------------------------- */
            double   y_76_c0 = _mpy2(c0, y_76);
            double   y_54_c0 = _mpy2(c0, y_54);
            double   y_32_c0 = _mpy2(c0, y_32);
            double   y_10_c0 = _mpy2(c0, y_10);
            int      y_7_c0  = _hi(y_76_c0), y_6_c0  = _lo(y_76_c0);
            int      y_5_c0  = _hi(y_54_c0), y_4_c0  = _lo(y_54_c0);
            int      y_3_c0  = _hi(y_32_c0), y_2_c0  = _lo(y_32_c0);
            int      y_1_c0  = _hi(y_10_c0), y_0_c0  = _lo(y_10_c0);

            /* ---------------------------------------------------------------- */
            /*  Perform right two columns of matrix multiply.  We generate 4    */
            /*  pairs of outputs that will be mixed with the results of the     */
            /*  luma calculations.  Recall that each chroma pair is shared      */
            /*  across two luma values.  The results here are 16Q16.            */
            /* ---------------------------------------------------------------- */
            double   cr64_c1 = _mpy2(c1, cr64);
            double   cr20_c1 = _mpy2(c1, cr20);
            double   cb64_c4 = _mpy2(c4, cb64);
            double   cb20_c4 = _mpy2(c4, cb20);
            int      cr6_c1  = _hi(cr64_c1), cr4_c1  = _lo(cr64_c1);
            int      cr2_c1  = _hi(cr20_c1), cr0_c1  = _lo(cr20_c1);
            int      cb6_c4  = _hi(cb64_c4), cb4_c4  = _lo(cb64_c4);
            int      cb2_c4  = _hi(cb20_c4), cb0_c4  = _lo(cb20_c4);

            int      cg6     = _dotp2(_packh2(cr64, cb64), c3c2);
            int      cg4     = _dotp2(_pack2 (cr64, cb64), c3c2);
            int      cg2     = _dotp2(_packh2(cb20, cr20), c2c3);
            int      cg0     = _dotp2(_pack2 (cb20, cr20), c2c3);

            /* ---------------------------------------------------------------- */
            /*  Mix the chroma and the luma together to make RGB.               */
            /* ---------------------------------------------------------------- */
            int      r_7     = y_7_c0 + cr6_c1;
            int      r_6     = y_6_c0 + cr6_c1;
            int      r_5     = y_5_c0 + cr4_c1;
            int      r_4     = y_4_c0 + cr4_c1;
            int      r_3     = y_3_c0 + cr2_c1;
            int      r_2     = y_2_c0 + cr2_c1;
            int      r_1     = y_1_c0 + cr0_c1;
            int      r_0     = y_0_c0 + cr0_c1;

            int      g_7     = y_7_c0 + cg6;
            int      g_6     = y_6_c0 + cg6;
            int      g_5     = y_5_c0 + cg4;
            int      g_4     = y_4_c0 + cg4;
            int      g_3     = y_3_c0 + cg2;
            int      g_2     = y_2_c0 + cg2;
            int      g_1     = y_1_c0 + cg0;
            int      g_0     = y_0_c0 + cg0;

            int      b_7     = y_7_c0 + cb6_c4;
            int      b_6     = y_6_c0 + cb6_c4;
            int      b_5     = y_5_c0 + cb4_c4;
            int      b_4     = y_4_c0 + cb4_c4;
            int      b_3     = y_3_c0 + cb2_c4;
            int      b_2     = y_2_c0 + cb2_c4;
            int      b_1     = y_1_c0 + cb0_c4;
            int      b_0     = y_0_c0 + cb0_c4;

            /* ---------------------------------------------------------------- */
            /*  Keep only the integer portion of the 16Q16 data, and pack down  */
            /*  to 8-bit bytes.                                                 */
            /* ---------------------------------------------------------------- */
            unsigned r_7654  = _spacku4(_packh2(r_7, r_6), _packh2(r_5, r_4));
            unsigned r_3210  = _spacku4(_packh2(r_3, r_2), _packh2(r_1, r_0));
            unsigned g_7654  = _spacku4(_packh2(g_7, g_6), _packh2(g_5, g_4));
            unsigned g_3210  = _spacku4(_packh2(g_3, g_2), _packh2(g_1, g_0));
            unsigned b_7654  = _spacku4(_packh2(b_7, b_6), _packh2(b_5, b_4));
            unsigned b_3210  = _spacku4(_packh2(b_3, b_2), _packh2(b_1, b_0));

            /* ---------------------------------------------------------------- */
            /*  Mask away LSBs to map to 5:6:5 gamut.                           */
            /* ---------------------------------------------------------------- */
            unsigned r_7654_ = r_7654 & 0xF8F8F8F8, r_3210_ = r_3210 & 0xF8F8F8F8;
            unsigned g_7654_ = g_7654 & 0xFCFCFCFC, g_3210_ = g_3210 & 0xFCFCFCFC;
            unsigned b_7654_ = b_7654 & 0xF8F8F8F8, b_3210_ = b_3210 & 0xF8F8F8F8;

            /* ---------------------------------------------------------------- */
            /*  Unpack the bytes to halfwords, aligning the red, green and      */
            /*  blue fields so that we can merge these together as 16-bit       */
            /*  5:6:5 pixels.  For Red, we need to unpack and shift left by 8.  */
            /*  For Green, unpack and shift left by 3.  For Blue, unpack and    */
            /*  shift right 3.  We do the Red and Blue shifts in two steps.     */
            /* ---------------------------------------------------------------- */
            double   r_7654u = _mpyu4(r_7654_, 0x80808080);             /* << 7 */
            double   r_3210u = _mpyu4(r_3210_, 0x80808080);             /* << 7 */
            double   g_7654u = _mpyu4(g_7654_, 0x08080808);             /* << 3 */
            double   g_3210u = _mpyu4(g_3210_, 0x08080808);             /* << 3 */
            double   b_7654u = _mpyu4(_rotl(b_7654_, 29), 0x01010101);  /* >> 6 */
            double   b_3210u = _mpyu4(_rotl(b_3210_, 29), 0x01010101);  /* >> 6 */
            unsigned r_76u   = _hi(r_7654u)*2, r_54u = _lo(r_7654u)*2;  /* << 1 */
            unsigned r_32u   = _hi(r_3210u)*2, r_10u = _lo(r_3210u)*2;  /* << 1 */
            unsigned g_76u   = _hi(g_7654u),   g_54u = _lo(g_7654u);
            unsigned g_32u   = _hi(g_3210u),   g_10u = _lo(g_3210u);
            unsigned b_76u   = _hi(b_7654u),   b_54u = _lo(b_7654u);
            unsigned b_32u   = _hi(b_3210u),   b_10u = _lo(b_3210u);

            /* ---------------------------------------------------------------- */
            /*  Merge together the R, G, and B fields.                          */
            /* ---------------------------------------------------------------- */
            unsigned rgb_76  = r_76u + g_76u + b_76u;
            unsigned rgb_54  = r_54u + g_54u + b_54u;
            unsigned rgb_32  = r_32u + g_32u + b_32u;
            unsigned rgb_10  = r_10u + g_10u + b_10u;

            // unsigned rgb_7 = rgb_76 & 0xFFFF0000;
            unsigned rgb_6 = rgb_76 & 0x0000FFFF;
            // unsigned rgb_5 = rgb_54 & 0xFFFF0000;
            unsigned rgb_4 = rgb_54 & 0x0000FFFF;
            // unsigned rgb_3 = rgb_32 & 0xFFFF0000;
            unsigned rgb_2 = rgb_32 & 0x0000FFFF;
            // unsigned rgb_1 = rgb_10 & 0xFFFF0000;
            unsigned rgb_0 = rgb_10 & 0x0000FFFF;

            // R's should be shifted by 11, but we want comparable to G

            // r_7 = (rgb_7 & 0xF800) >> 10;
            r_6 = (rgb_6 & 0xF800) >> 10;
            // r_5 = (rgb_5 & 0xF800) >> 10;
            r_4 = (rgb_4 & 0xF800) >> 10;
            // r_3 = (rgb_3 & 0xF800) >> 10;
            r_2 = (rgb_2 & 0xF800) >> 10;
            // r_1 = (rgb_1 & 0xF800) >> 10;
            r_0 = (rgb_0 & 0xF800) >> 10;

            // g_7 = (rgb_7 & 0x07E0) >> 5;
            g_6 = (rgb_6 & 0x07E0) >> 5;
            // g_5 = (rgb_5 & 0x07E0) >> 5;
            g_4 = (rgb_4 & 0x07E0) >> 5;
            // g_3 = (rgb_3 & 0x07E0) >> 5;
            g_2 = (rgb_2 & 0x07E0) >> 5;
            // g_1 = (rgb_1 & 0x07E0) >> 5;
            g_0 = (rgb_0 & 0x07E0) >> 5;

            // signed diff_7 = r_7 - g_7;
            signed diff_6 = r_6 - g_6;
            // signed diff_5 = r_5 - g_5;
            signed diff_4 = r_4 - g_4;
            // signed diff_3 = r_3 - g_3;
            signed diff_2 = r_2 - g_2;
            // signed diff_1 = r_1 - g_1;
            signed diff_0 = r_0 - g_0;

            unsigned char isSkin_0 = diff_0 > min_thresh && diff_0 < max_thresh ? 1 : 0;
            unsigned char isSkin_2 = diff_2 > min_thresh && diff_2 < max_thresh ? 1 : 0;
            unsigned char isSkin_4 = diff_4 > min_thresh && diff_4 < max_thresh ? 1 : 0;
            unsigned char isSkin_6 = diff_6 > min_thresh && diff_6 < max_thresh ? 1 : 0;

            unsigned background = j < num_pixels / 2 ? p1_color : p2_color;
            unsigned mirror_j = num_pixels - j - 1;

            unsigned color_0 = isSkin_0 ? skin_color : background;
            _amem2(&rgb_data_p[mirror_j - 0]) = color_0;
            _amem2(&rgb_data_p[mirror_j - 1]) = color_0;

			unsigned color_2 = isSkin_2 ? skin_color : background;
			_amem2(&rgb_data_p[mirror_j - 2]) = color_2;
			_amem2(&rgb_data_p[mirror_j - 3]) = color_2;

			unsigned color_4 = isSkin_4 ? skin_color : background;
			_amem2(&rgb_data_p[mirror_j - 4]) = color_4;
			_amem2(&rgb_data_p[mirror_j - 5]) = color_4;

			unsigned color_6 = isSkin_6 ? skin_color : background;
			_amem2(&rgb_data_p[mirror_j - 6]) = color_6;
			_amem2(&rgb_data_p[mirror_j - 7]) = color_6;


            unsigned char* player;
            int offset;
            if(j < player_width){
            	player = player2;
            	offset = 0;
            }
            else{
            	player = player1;
            	offset = player_width;
            }

            player[(i*player_width)-offset+j+7] = isSkin_6;
            player[(i*player_width)-offset+j+6] = isSkin_6;
            player[(i*player_width)-offset+j+5] = isSkin_4;
            player[(i*player_width)-offset+j+4] = isSkin_4;
            player[(i*player_width)-offset+j+3] = isSkin_2;
            player[(i*player_width)-offset+j+2] = isSkin_2;
            player[(i*player_width)-offset+j+1] = isSkin_0;
            player[(i*player_width)-offset+j+0] = isSkin_0;
         }
     }

     // Draw center dots
     unsigned short * rgb = (unsigned short *) rgb_data;
     for(i = (num_lines / 2) - 5; i < (num_lines / 2) + 5; i++){
		 for(j = (player_width / 2) - 5; j < (player_width / 2) + 5; j++){
			 _amem2(&rgb[i*num_pixels + j]) = 0x6FE6;
			 _amem2(&rgb[i*num_pixels + j + player_width]) = 0x6FE6;
		 }
	 }

     int k, l;

     // Draw P1 R, P, or S
     if(RPS_disp_p1){
		 for(i = 0; i < 6; ++i){
			 for(j = 0; j < 6; ++j){
				 if(RPS_disp_p1[i*6 + j]){
					 for(k = 0; k < 4; ++k){
						 for(l = 0; l < 4; ++l){
							 _amem2(&rgb[(4*i+k+20)*num_pixels + (4*j+l+20)]) = 0x6FE6;
						 }
					 }
				 }
			 }
		 }
     }

     // Draw P2 R, P, or S
     if(RPS_disp_p2){
		 for(i = 0; i < 6; ++i){
			 for(j = 0; j < 6; ++j){
				 if(RPS_disp_p2[i*6 + j]){
					 for(k = 0; k < 4; ++k){
						 for(l = 0; l < 4; ++l){
							 _amem2(&rgb[(4*i+k+20)*num_pixels + (4*j+l+600)]) = 0x6FE6;
						 }
					 }
				 }
			 }
		 }
	  }

     // Draw P1 and P2 scores
     for(i = 0; i < 7; ++i){
    	 for(j = 0; j < 5; ++j){
    		 if(score_disp_p1[i*5 + j]){
    			 for(k = 0; k < 4; ++k){
    				 for(l = 0; l < 4; l++){
    					 _amem2(&rgb[(4*i+k+430)*num_pixels + (4*j+l+20)]) = 0x6FE6;
    				 }
    			 }
    		 }
    		 if(score_disp_p2[i*5 + j]){
				 for(k = 0; k < 4; ++k){
					 for(l = 0; l < 4; l++){
						 _amem2(&rgb[(4*i+k+430)*num_pixels + (4*j+l+600)]) = 0x6FE6;
					 }
				 }
			 }
    	 }
     }
}
#endif
