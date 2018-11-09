/**
 * \file  vpiflcd_loopback.c
 *
 * \brief Sample application for capture from VPIF and display over LCD.
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

#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include "psc.h"
#include "vpif.h"
#include "raster.h"
#include "interrupt.h"

#include "lcdkC6748.h"
#include "soc_C6748.h"
#include "hw_psc_C6748.h"
#include "hw_syscfg0_C6748.h"
#include "hw_types.h"

#include "adv7343.h"
#include "tvp5147.h"
#include "cdce913.h"
#include "codecif.h"
#include "i2cgpio.h"

#ifdef _TMS320C6X
#include "dspcache.h"
#else
#include "cp15.h"
#ifdef gcc
#define restrict
#endif
#endif

/******************************************************************************
**                      INTERNAL MACROS
*******************************************************************************/
#define I2C_SLAVE_CODEC_ADV7343             (0x2Au)
#define I2C_SLAVE_CODEC_TVP5147_1_SVIDEO    (0x5Cu)
#define I2C_SLAVE_CODEC_TVP5147_2_COMPOSITE (0x5Du)
#define I2C_SLAVE_CDCE913                   (0x65u)
#define I2C_SLAVE_UI_EXPANDER               (0x20u)

#define INT_CHANNEL_I2C                     (0x2u)

#define CAPTURE_IMAGE_WIDTH                 (720)
#define CAPTURE_IMAGE_HEIGHT                (488)

#define DISPLAY_IMAGE_WIDTH                 (640)
#define DISPLAY_IMAGE_HEIGHT                (480)

#define OFFSET1                             ((CAPTURE_IMAGE_HEIGHT - DISPLAY_IMAGE_HEIGHT)/2)
#define OFFSET2                             ((CAPTURE_IMAGE_WIDTH - DISPLAY_IMAGE_WIDTH)/2)
#define OFFSET                              (CAPTURE_IMAGE_WIDTH * OFFSET1 + OFFSET2)

/******************************************************************************
**                      GLOBAL FUNCTION PROTOTYPES
*******************************************************************************/

extern void cbcr422sp_to_rgb565_c(const unsigned char * restrict, unsigned int,
        unsigned int, const short*, const unsigned char *restrict,
        unsigned int, unsigned short *restrict, unsigned int, unsigned,
        unsigned char*, unsigned char*, unsigned char*, unsigned char*, unsigned char*, unsigned char*);

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void SetupIntc(void);
static void SetUpVPIFRx(void);
static void SetUpLCD(void);
static void VPIFIsr(void);
static void LCDIsr(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
unsigned int            i, pingpong=1, buffcount=0, buffcount2, display_buff_1=0;
volatile unsigned int   captured=0, changed=0, updated=3, processed=1;
unsigned char           buff_luma1[CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT];
unsigned char           buff_luma2[CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT];
unsigned char           buff_chroma1[CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT];
unsigned char           buff_chroma2[CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT];
unsigned char           *buff_luma[2], *buff_chroma[2];
unsigned char           *videoTopY, *videoTopC;
unsigned short          *videoTopRgb1, *videoTopRgb2;
unsigned short          Rgb_buffer1[DISPLAY_IMAGE_WIDTH*DISPLAY_IMAGE_HEIGHT + 16];
unsigned short          Rgb_buffer2[DISPLAY_IMAGE_WIDTH*DISPLAY_IMAGE_HEIGHT + 16];
const short             ccCoeff[5] = {0x2000, 0x2BDD, -0x0AC5, -0x1658, 0x3770};

/******************************************************************************
**                      IMAGE DETECTION VARIABLE AND FUNCTION DEFINITIONS
*******************************************************************************/
unsigned char player1[DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)]={0};
unsigned char player2[DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)]={0};
unsigned char p1_filtered[DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)]={0};
unsigned char p2_filtered[DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)]={0};
float p1_features[16] = {0};
float p2_features[16] = {0};

int p1_score = 0;
int p2_score = 0;

// Queue placed in external memory
int queue_arr[DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)];
#pragma DATA_SECTION(queue_arr,".EXT_RAM")

const int player_height = DISPLAY_IMAGE_HEIGHT;
const int player_width = DISPLAY_IMAGE_WIDTH / 2;

// Player hand boundaries
int max_i_1, min_i_1;
int max_j_1, min_j_1;
int max_i_2, min_i_2;
int max_j_2, min_j_2;

// Buffer variables
int wait = 1;
int SWITCH = 1;

// BFS globals
// These arrays are used to get row and column numbers of 8 neighbors
// of a given cell
static int iOffset[] = {-1, -1, -1,  0, 0,  1, 1, 1};
static int jOffset[] = {-1,  0,  1, -1, 1, -1, 0, 1};

// Filtering function declarations
void clearFiltered(unsigned char*);
int canVisit(unsigned char*, int, int, unsigned char*);
void BFS(unsigned char*, int, int, unsigned char*, int*, int*, int*, int*);
void extractFeatures(unsigned char*, int*, int*, int*, int*, float*);

/******************************************************************************
**                      NEURAL NETWORK VARIABLE AND FUNCTION DEFINITIONS
*******************************************************************************/
#define INPUT_N 16
#define L1_N 7
#define L2_N 3

// Pre-processing constants
float xoffset[INPUT_N] = {0,0,0,0,0,0.2418864,0.1670359,0.03898981,0,0.2465035,0.09046052,0.01224105,0,0,0,0};
float gain[INPUT_N] = {2.74541279271763,2.18469665869216,2.13112628212553,2.51415089833126,2.10745224618583,2.63812705642004,2.40106386337659,2.08114338517056,2.07476627757883,2.65429235570437,2.19891499377245,2.02478550055153,2.63157894736842,2.02086053495816,2,2.07058821580623};
int ymin = -1;

// Layer 1
float b1[L1_N] = {-1.5848286611197138,0.56914406247483929,-0.4083323481141915,1.1328763970319213,1.2222248290752669,0.43748108728668234,2.696324462986047};
float IW1_1[L1_N][INPUT_N] = {{-0.66007053527896487,-1.6156879060339895,-0.81724096039895333,0.11044018156002589,0.34191901673799513,0.12232728556042174,-0.077107602191385766,-0.41273573875985003,-1.3378606556178327,-0.63382426477538301,-0.0616402370917375,-1.5450228505790125,0.85400680831030507,-1.6005421093340675,0.59856375611711143,-0.17832883516788472},
							  {-0.025582764710409062,1.4736568665427461,-1.0051522180922863,-0.39942642572382558,1.8561444571800856,0.63107402249139943,-0.94095931811861766,-0.022909070687601656,1.7613549635590122,-0.34726075398374878,-0.98542522487273099,0.15168431053866813,0.96020624964134271,1.7078242827037295,0.041506507420837141,-0.37728250556969556},
							  {-1.1514467229523671,0.022798142203842887,-0.7083216392888394,0.87451235116898085,-0.097541031218355778,-0.095926801237243634,-0.71538723353544564,-0.96231938888554602,-2.4849755791453525,-0.51636257890378479,0.34524886899793694,-0.32577816530455045,-1.0087046336621168,-0.84494410866387992,0.54436936353337073,-1.0455870364337299},
							  {0.091071857574146403,-1.5003687018144987,0.32655189281519081,0.65150100296332658,-0.96959661704490718,-0.091028430946902314,0.29839833439503344,-2.0269035650025118,-0.55036230914472639,1.4019799874316281,-0.013560162286470247,-1.71950513105785,2.7810686979311181,-0.68266486535869919,0.26755364196340786,1.6842710311635676},
							  {0.02713649633365061,-0.22280425583287425,-0.3686082480096507,-0.1162452916722356,0.54067508848748791,0.01576491143725332,0.094772621352086847,-1.0404684223266938,1.3383287209392705,0.60511639687697771,0.060477563010154604,-0.32517640565401773,1.1741210123322896,-0.3134943333160945,-0.62490862819166249,-0.31722018719067369},
							  {0.6847207431202319,-0.80896428829806344,-0.50804019462815775,0.41975529021028046,-0.83551153686927149,-0.69746898347629072,-0.24572360124279616,0.33113445532069652,-1.3198870559089124,-0.53951134436158465,0.61870619268773441,-0.31390013040528686,-1.146907615149368,-0.83313873121727589,0.85848318748179342,0.70370445683353222},
							  {0.32936313270470707,-1.2596143375580795,0.25966240119662293,0.79010713762825324,0.38377268237398715,-0.24251574105118828,-0.44195947391332979,-0.24675350992341005,0.96422438160890289,1.1820795509733257,-0.6997177754045294,-1.8555382741187418,1.8844669107713186,-2.4170721148829015,-0.57391558792469632,1.5064648460746477}};

// Layer 2
float b2[L2_N] = {-1.0345552046584343,1.1644763555268924,0.45379009215275906};
float LW2_1[L2_N][L1_N] = {{-0.83962594072839103,-1.3899488207233714,3.9556588372733232,-3.7455356856337403,-1.9829617546940401,2.6827171508675476,-3.8570706881050816},
						   {2.433873767267595,-3.6567510002984966,-1.7640852425927975,1.9778900303540849,-0.35536343222102706,0.19139025786875657,3.6308826292768539},
						   {-0.49558293631668532,4.9105009018756771,-0.43581036416892505,-0.1641455591897773,0.32651453602687697,-2.011095789230839,-0.050625579468620306}};

float y[INPUT_N];
float layer1[L1_N];
float layer2[L2_N];
float output_p1[L2_N];
float output_p2[L2_N];
int result_p1 = -1;
int result_p2 = -1;
int prev_res_p1 = -1;
int prev_res_p2 = -1;

// Neural net function declarations
float sigmoid(float);
int neuralNetwork(float*, float*);

/******************************************************************************
**                      RESULT DISPLAY VARIABLE AND FUNCTION DEFINITIONS
*******************************************************************************/
unsigned char* RPS_disp_p1;
unsigned char* RPS_disp_p2;
unsigned char* score_disp_p1;
unsigned char* score_disp_p2;

unsigned char P[] = {1,1,1,1,1,0,
					 1,0,0,0,0,1,
					 1,1,1,1,1,0,
					 1,0,0,0,0,0,
					 1,0,0,0,0,0,
					 1,0,0,0,0,0};

unsigned char S[] = {0,1,1,1,1,1,
					 1,0,0,0,0,0,
					 0,1,1,0,0,0,
					 0,0,0,1,1,0,
					 0,0,0,0,0,1,
					 1,1,1,1,1,0};

unsigned char R[] = {1,1,1,1,1,0,
					 1,0,0,0,0,1,
					 1,1,1,1,1,0,
					 1,0,0,1,0,0,
				   	 1,0,0,0,1,0,
					 1,0,0,0,0,1};

unsigned char zero[] =   {0,1,1,1,0,
						  1,0,0,0,1,
						  1,0,0,0,1,
						  1,0,0,0,1,
						  1,0,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0};

unsigned char one[] =    {0,0,1,0,0,
		                  0,1,1,0,0,
					      0,0,1,0,0,
					      0,0,1,0,0,
					      0,0,1,0,0,
					      0,0,1,0,0,
					      0,1,1,1,0};

unsigned char two[] =    {0,1,1,1,0,
        				  1,0,0,0,1,
        				  0,0,0,1,1,
        				  0,0,1,1,0,
        				  0,1,1,0,0,
        				  1,1,0,0,0,
        				  1,1,1,1,1};

unsigned char three[] =  {0,1,1,1,0,
						  1,0,0,0,1,
						  0,0,0,0,1,
						  0,1,1,1,0,
						  0,0,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0};

unsigned char four[] =   {0,0,1,1,0,
						  0,1,0,1,0,
						  1,0,0,1,0,
						  1,1,1,1,1,
						  0,0,0,1,0,
						  0,0,0,1,0,
						  0,0,0,1,0};

unsigned char five[] =   {1,1,1,1,1,
						  1,0,0,0,0,
						  0,1,1,1,0,
						  0,0,0,1,1,
						  0,0,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0};

unsigned char six[] =    {0,1,1,1,0,
						  1,0,0,0,1,
						  1,0,0,0,0,
						  1,0,1,1,0,
						  1,1,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0};

unsigned char seven[] =  {1,1,1,1,1,
						  0,0,0,0,1,
						  0,0,0,1,1,
						  0,0,1,1,0,
						  0,1,1,0,0,
						  1,1,0,0,0,
						  1,0,0,0,0};

unsigned char eight[] =  {0,1,1,1,0,
						  1,0,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0,
						  1,0,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0};

unsigned char nine[] =   {0,1,1,1,0,
						  1,0,0,0,1,
						  1,0,0,1,1,
						  0,1,1,0,1,
						  0,0,0,0,1,
						  1,0,0,0,1,
						  0,1,1,1,0};



void updateDisplayRPS(int, unsigned char**);
void updateDisplayScore(int, unsigned char**);

/******************************************************************************
**                      QUEUE DEFINITIONS FOR DFS
*******************************************************************************/

// A structure to represent a queue
struct Queue
{
    int front, rear, size;
    unsigned capacity;
    int* array;
};

// function to create a queue of given capacity.
// It initializes size of queue as 0
struct Queue* createQueue(unsigned capacity)
{
    struct Queue* queue = (struct Queue*) malloc(sizeof(struct Queue));
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;  // This is important, see the enqueue
    // int arr[DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)] = {0};
    queue->array = queue_arr; // (int*) malloc(queue->capacity * sizeof(int));
    return queue;
}

// Queue is full when size becomes equal to the capacity
int isFull(struct Queue* queue)
{  return (queue->size == queue->capacity);  }

// Queue is empty when size is 0
int isEmpty(struct Queue* queue)
{  return (queue->size == 0); }

// Function to add an item to the queue.
// It changes rear and size
void enqueue(struct Queue* queue, int item)
{
    if (isFull(queue))
        return;
    queue->rear = (queue->rear + 1)%queue->capacity;
    queue->array[queue->rear] = item;
    queue->size = queue->size + 1;
    //printf("%d enqueued to queue\n", item);
}

// Function to remove an item from queue.
// It changes front and size
int dequeue(struct Queue* queue)
{
    if (isEmpty(queue))
        return INT_MIN;
    int item = queue->array[queue->front];
    queue->front = (queue->front + 1)%queue->capacity;
    queue->size = queue->size - 1;
    //printf("%d dequeued from queue\n", item);
    return item;
}

// Function to get front of queue
int front(struct Queue* queue)
{
    if (isEmpty(queue))
        return INT_MIN;
    return queue->array[queue->front];
}

// Function to get rear of queue
int rear(struct Queue* queue)
{
    if (isEmpty(queue))
        return INT_MIN;
    return queue->array[queue->rear];
}

void resetQueue(struct Queue* queue)
{
	queue->front = queue->size = 0;
	queue->rear = queue->capacity - 1;  // This is important, see the enqueue
}

struct Queue* queue;

/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#elif _TMS320C6X
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/
int main(void)
{
#ifndef _TMS320C6X
    unsigned int index;
#endif

    int i;
    for(i = 0; i < (DISPLAY_IMAGE_HEIGHT*(DISPLAY_IMAGE_WIDTH/2)); i++){
    	player1[i]=42;
    	player2[i]=42;
    }

    /* Setting the Master priority for the VPIF and LCD DMA controllers to highest level */
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_MSTPRI1) &= 0x00FFFFFF;
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_MSTPRI2) &= 0x0FFFFFFF;

#ifdef _TMS320C6X
    /* Set MAR bits and configure L1 cache */
    CacheEnableMAR((unsigned int)0xC0000000, (unsigned int)0x10000000);
    CacheEnable(L1PCFG_L1PMODE_32K | L1DCFG_L1DMODE_32K );
#else
    /* Sets up 'Level 1" page table entries.
     * The page table entry consists of the base address of the page
     * and the attributes for the page. The following operation is to
     * setup one-to-one mapping page table for DDR memeory range and set
     * the atributes for the same. The DDR memory range is from 0xC0000000
     * to 0xCFFFFFFF. Thus the base of the page table ranges from 0xC00 to
     * 0xCFF. Cache(C bit) and Write Buffer(B bit) are enabled  only for
     * those page table entries which maps to DDR RAM and internal RAM.
     * All the pages in the DDR range are provided with R/W permissions */
    for(index = 0; index < (4*1024); index++)
    {
         if((index >= 0xC00 && index < 0xD00)|| (index == 0x800))
         {
              pageTable[index] = (index << 20) | 0x00000C1E;
         }
         else
         {
              pageTable[index] = (index << 20) | 0x00000C12;
         }
    }

    /* Configures translation table base register
     * with pagetable base address. */
    CP15TtbSet((unsigned int )pageTable);

    /* Enables MMU */
    CP15MMUEnable();

    /* Enable Instruction Cache */
    CP15ICacheEnable();

    /* Enable Data Cache */
    CP15DCacheEnable();
#endif

    /* Allocate pointers to buffers */
    buff_luma[0] = buff_luma1;
    buff_luma[1] = buff_luma2;
    buff_chroma[0] = buff_chroma1;
    buff_chroma[1] = buff_chroma2;

    /* Initializing palette for first buffer */
    Rgb_buffer1[0] = 0x4000;
    for (i = 1; i < 16; i++)
        Rgb_buffer1[i] = 0x0000;
    videoTopRgb1 = Rgb_buffer1 + i;

    /* Initializing palette for second buffer */
    Rgb_buffer2[0] = 0x4000;
    for (i = 1; i < 16; i++)
        Rgb_buffer2[i] = 0x0000;
    videoTopRgb2 = Rgb_buffer2 + i;

    /* Power on VPIF */
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_VPIF, PSC_POWERDOMAIN_ALWAYS_ON,
            PSC_MDCTL_NEXT_ENABLE);

    /* Initializing ARM/DSP INTC */
    SetupIntc();

    /* Initialize I2C and program UI GPIO expander, TVP5147, and ADV7343 via I2C */
    I2CPinMuxSetup(0);

    /*Initialize the TVP5147 to accept composite video */
    I2CCodecIfInit(SOC_I2C_0_REGS, INT_CHANNEL_I2C,
            I2C_SLAVE_CODEC_TVP5147_2_COMPOSITE);
    TVP5147CompositeInit(SOC_I2C_0_REGS);

    /* Setup VPIF pinmux */
    VPIFPinMuxSetup();

    /* Setup LCD */
    SetUpLCD();

    /* Initialize VPIF */
    SetUpVPIFRx();
    VPIFDMARequestSizeConfig(SOC_VPIF_0_REGS, VPIF_REQSIZE_ONE_TWENTY_EIGHT);
    VPIFEmulationControlSet(SOC_VPIF_0_REGS, VPIF_HALT);

    /* Initialize buffer addresses for 1st frame*/
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_TOP_FIELD,
            VPIF_LUMA, (unsigned int) buff_luma[0], CAPTURE_IMAGE_WIDTH*2);
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_TOP_FIELD,
            VPIF_CHROMA, (unsigned int) buff_chroma[0], CAPTURE_IMAGE_WIDTH*2);
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_BOTTOM_FIELD,
            VPIF_LUMA, (unsigned int) (buff_luma[0] + CAPTURE_IMAGE_WIDTH), CAPTURE_IMAGE_WIDTH*2);
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_BOTTOM_FIELD,
            VPIF_CHROMA, (unsigned int) (buff_chroma[0] + CAPTURE_IMAGE_WIDTH), CAPTURE_IMAGE_WIDTH*2);

    /* configuring the base ceiling */
    RasterDMAFBConfig(SOC_LCDC_0_REGS, (unsigned int) Rgb_buffer2,
            (unsigned int) (Rgb_buffer2 + DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT + 15), 0);
    RasterDMAFBConfig(SOC_LCDC_0_REGS, (unsigned int) Rgb_buffer2,
            (unsigned int) (Rgb_buffer2 + DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT + 15), 1);

    /* Enable capture */
    VPIFCaptureChanenEnable(SOC_VPIF_0_REGS, VPIF_CHANNEL_0);

    /* Enable VPIF interrupt */
    VPIFInterruptEnable(SOC_VPIF_0_REGS, VPIF_FRAMEINT_CH0);
    VPIFInterruptEnableSet(SOC_VPIF_0_REGS, VPIF_FRAMEINT_CH0);

    /* enable End of frame interrupt */
    RasterEndOfFrameIntEnable(SOC_LCDC_0_REGS);

    /* enable raster */
    RasterEnable(SOC_LCDC_0_REGS);

    buffcount++;
    buffcount2 = buffcount - 1;

    queue = createQueue(player_width*player_height);

    /* Run forever */
    while (1)
    {
        /* Wait here till a new frame is not captured */
        while (!captured);

        /* Process the next buffer only when both the raster buffers
         * are pointing to the current buffer to avoid jitter effect */
        if (updated == 3)
        {
            processed = 0;
            changed = 0;
            updated = 0;

         /* Convert the buffer from CBCR422 semi-planar to RGB565,
          *  Flush and invalidate the processed buffer so that the DMA reads the processed data,
          *  set the flag for the buffer to be displayed on the LCD (which would be the processed buffer)
          *  and notify the LCD of availability of a processed buffer.
          *  The output buffers are ping-ponged each time. */
            if (pingpong)
            {
                cbcr422sp_to_rgb565_c(
                        (const unsigned char *) (videoTopC + OFFSET),
                        DISPLAY_IMAGE_HEIGHT, CAPTURE_IMAGE_WIDTH, ccCoeff,
                        (const unsigned char *) (videoTopY + OFFSET),
                        CAPTURE_IMAGE_WIDTH, videoTopRgb1, DISPLAY_IMAGE_WIDTH, DISPLAY_IMAGE_WIDTH,
                        player1, player2, RPS_disp_p1, RPS_disp_p2, score_disp_p1, score_disp_p2);
            #ifdef _TMS320C6X
                CacheWBInv((unsigned int) videoTopRgb1, DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT * 2);
            #else
                CP15DCacheCleanBuff((unsigned int) videoTopRgb1,DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT * 2);
            #endif
                display_buff_1 = 1;
                changed = 1;
            }
            else
            {
                cbcr422sp_to_rgb565_c(
                        (const unsigned char *) (videoTopC + OFFSET),
                        DISPLAY_IMAGE_HEIGHT, CAPTURE_IMAGE_WIDTH, ccCoeff,
                        (const unsigned char *) (videoTopY + OFFSET),
                        CAPTURE_IMAGE_WIDTH, videoTopRgb2, DISPLAY_IMAGE_WIDTH, DISPLAY_IMAGE_WIDTH,
                        player1, player2, RPS_disp_p1, RPS_disp_p2, score_disp_p1, score_disp_p2);
            #ifdef _TMS320C6X
                CacheWBInv((unsigned int) videoTopRgb2, DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT * 2);
            #else
                CP15DCacheCleanBuff((unsigned int) videoTopRgb2, DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT * 2);
            #endif
                display_buff_1 = 0;
                changed = 1;
            }


            if(SWITCH){//wait == 0){
            	// Reset boundary variables
            	max_i_1 = INT_MIN;
            	min_i_1 = INT_MAX;
            	max_j_1 = INT_MIN;
            	min_j_1 = INT_MAX;
            	max_i_2 = INT_MIN;
            	min_i_2 = INT_MAX;
            	max_j_2 = INT_MIN;
            	min_j_2 = INT_MAX;

            	// Reset Display Arrays
            	// RPS_disp_p1 = RPS_disp_p2 = NULL;

				// BFS from middle
				clearFiltered(p1_filtered);
				clearFiltered(p2_filtered);

				BFS(player1, player_height / 2, player_width / 2, p1_filtered, &max_i_1, &min_i_1, &max_j_1, &min_j_1);
				BFS(player2, player_height / 2, player_width / 2, p2_filtered, &max_i_2, &min_i_2, &max_j_2, &min_j_2);
				extractFeatures(p1_filtered, &max_i_1, &min_i_1, &max_j_1, &min_j_1, p1_features);
				extractFeatures(p2_filtered, &max_i_2, &min_i_2, &max_j_2, &min_j_2, p2_features);

				// If the boundaries are still INT_MIN, then no image is being detected
				prev_res_p1 = result_p1;
				prev_res_p2 = result_p2;
				result_p1 = max_i_1 != INT_MIN ? neuralNetwork(p1_features, output_p1) : -1;
				result_p2 = max_i_2 != INT_MIN ? neuralNetwork(p2_features, output_p2) : -1;

				if(result_p1 != -1 && result_p2 != -1 && !(prev_res_p1 == result_p1 && prev_res_p2 == result_p2)){
					switch(result_p1){
						case 0:
							switch(result_p2){
								case 1:
									p2_score++;
									break;
								case 2:
									p1_score++;
									break;
							}
							break;
						case 1:
							switch(result_p2){
								case 0:
									p1_score++;
									break;
								case 2:
									p2_score++;
									break;
							}
							break;
						case 2:
							switch(result_p2){
								case 0:
									p2_score++;
									break;
								case 1:
									p1_score++;
									break;
							}
							break;
					}

					// Reset scores at 10
					if(p1_score >= 10 || p2_score >= 10) p1_score = p2_score = 0;
				}

				// Updates rock or paper or scissor
				updateDisplayRPS(result_p1, &RPS_disp_p1);
				updateDisplayRPS(result_p2, &RPS_disp_p2);

				// Updates score on screen
				updateDisplayScore(p1_score, &score_disp_p1);
				updateDisplayScore(p2_score, &score_disp_p2);
            }

            pingpong = !pingpong;
            captured = 0;
            processed = 1;
        }
    }
}

// Reset the filtered arrays
void clearFiltered(unsigned char* filtered){
	int i;
	for(i = 0; i < player_width*player_height; i++){
		filtered[i] = 0;
	}
}

// A function to check if a given cell (i, j) can be included in DFS
int canVisit(unsigned char* player, int i, int j, unsigned char* filtered)
{
    // row number is in range, column number is in range and value is 1
    // and not yet visited
    return (i >= 0) && (i < player_height) &&
           (j >= 0) && (j < player_width) &&
           (player[i*player_width + j] == 1) && (filtered[i*player_width + j] != 1);
}

// A utility function to do Breadth First Search for a 2D player matrix.
// It only considers the 8 neighbors as adjacent vertices
void BFS(unsigned char* player, int i, int j, unsigned char* filtered, int* max_i, int* min_i, int* max_j, int* min_j){
	resetQueue(queue);

	// Mark this cell as visited
	if(canVisit(player, i, j, filtered)){
		enqueue(queue, i*player_width + j);
		filtered[i*player_width + j] = 1;
	}

	while(!isEmpty(queue)){
		int coord = dequeue(queue);
		int cur_i = coord / player_width;
		int cur_j = coord % player_width;

		// Iterate for all safe connected neighbors
		int k;
		for (k = 0; k < 8; ++k){
			int new_i = cur_i + iOffset[k];
			int new_j = cur_j + jOffset[k];
			if (canVisit(player, new_i, new_j, filtered)){
				int new_coord = new_i*player_width + new_j;
				enqueue(queue, new_coord);
				filtered[new_coord] = 1;
				if(new_i > *max_i) *max_i = new_i;
				if(new_i < *min_i) *min_i = new_i;
				if(new_j > *max_j) *max_j = new_j;
				if(new_j < *min_j) *min_j = new_j;


				// printf("Enqueued %d %d\n", new_i, new_j);
			}
		}
	}
}

// Extracts feature vector from filtered image
// Add checks for min and max i and j values
void extractFeatures(unsigned char* filtered, int* max_i, int* min_i, int* max_j, int* min_j, float* features){
	int res = 4;

	// Check if ranges are correct
	if(*max_i < *min_i || *max_j < *min_j) return;
	int i_range = (*max_i - *min_i) / res;
	int j_range = (*max_j - *min_j) / res;

	int k, i, j, i_step, j_step, count_on;
	for(k = 0 ; k < res* res; ++k){
		i_step = k % res;
		j_step = k / res;
		count_on = 0;
		for(i = *min_i + i_step*i_range; i < *min_i + (i_step+1)*i_range; ++i){
			for(j = *min_j + j_step*j_range; j < *min_j + (j_step+1)*j_range; ++j){
				count_on += filtered[i*player_width + j];
			}
		}
		features[k] = i_range * j_range > 0 ? count_on / ((float)(i_range * j_range)) : 42;
	}
}

// Sigmoid for neural network
float sigmoid(float x){
	return ((2 / (1 + exp((-2)*x))) - 1);
}

int neuralNetwork(float* input, float* output){
	// Preprocessing
	for(i = 0; i < INPUT_N; i++){
		y[i] = ((input[i] - xoffset[i]) * gain[i]) + ymin;
	}

	// Layer 1
	for(i = 0; i < L1_N; i++){
		int j;
		float sig_in = 0;
		for(j = 0; j < INPUT_N; j++){
		  sig_in += (IW1_1[i][j]*y[j]);
		}
		sig_in +=  b1[i];
		layer1[i] = sigmoid(sig_in);
	}

	// Layer 2
	float sum_sm = 0;
	for(i = 0; i < L2_N; i++){
		int j;
		float sm_in = 0;
		for(j = 0; j < L1_N; j++){
		  sm_in += (LW2_1[i][j]*layer1[j]);
		}
		sm_in += b2[i];
		layer2[i] = exp(sm_in);
		sum_sm += exp(sm_in);
	}

	// Output
	int result = -1;
	float max = 0;
	for(i = 0; i < L2_N; i++){
		output[i] = layer2[i] / sum_sm;
		if(output[i] > max){
			max = output[i];
			result = i;
		}
	}

	return result;
}

void updateDisplayRPS(int result, unsigned char** display){
	switch(result){
		case 0:
			*display = P;
			break;
		case 1:
			*display = S;
			break;
		case 2:
			*display = R;
			break;
		default:
			*display = 0;
	}
}

void updateDisplayScore(int score, unsigned char** display){
	switch(score){
		case 0:
			*display = zero;
			break;
		case 1:
			*display = one;
			break;
		case 2:
			*display = two;
			break;
		case 3:
			*display = three;
			break;
		case 4:
			*display = four;
			break;
		case 5:
			*display = five;
			break;
		case 6:
			*display = six;
			break;
		case 7:
			*display = seven;
			break;
		case 8:
			*display = eight;
			break;
		case 9:
			*display = nine;
			break;
	}
}

/*
 * Initialize capture
 */
static void SetUpVPIFRx(void)
{
    /* Disable interrupts */
    VPIFInterruptDisable(SOC_VPIF_0_REGS, VPIF_FRAMEINT_CH1);
    VPIFInterruptDisable(SOC_VPIF_0_REGS, VPIF_FRAMEINT_CH0);

    /* Disable capture ports */
    VPIFCaptureChanenDisable(SOC_VPIF_0_REGS, VPIF_CHANNEL_1);
    VPIFCaptureChanenDisable(SOC_VPIF_0_REGS, VPIF_CHANNEL_0);

    /* Interrupt after capturing the bottom field of every frame */
    VPIFCaptureIntframeConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_FRAME_INTERRUPT_BOTTOM);

    /* Y/C interleaved capture over 8-bit bus */
    VPIFCaptureYcmuxModeSelect(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_YC_MUXED);

    /* Capturing 480I (SD NTSC) */
    VPIFCaptureModeConfig(SOC_VPIF_0_REGS, VPIF_480I, VPIF_CHANNEL_0, 0, (struct vbufParam *) 0);
}

/*
 * Configures raster to display image
 */
static void SetUpLCD(void)
{
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_LCDC, PSC_POWERDOMAIN_ALWAYS_ON,
            PSC_MDCTL_NEXT_ENABLE);

    LCDPinMuxSetup();

    /* disable raster */
    RasterDisable(SOC_LCDC_0_REGS);

    /* configure the pclk */
    RasterClkConfig(SOC_LCDC_0_REGS, 25000000, 150000000);

    /* configuring DMA of LCD controller */
    RasterDMAConfig(SOC_LCDC_0_REGS, RASTER_DOUBLE_FRAME_BUFFER,
                    RASTER_BURST_SIZE_16, RASTER_FIFO_THRESHOLD_8,
                    RASTER_BIG_ENDIAN_DISABLE);

    /* configuring modes(ex:tft or stn,color or monochrome etc) for raster controller */
    RasterModeConfig(SOC_LCDC_0_REGS, RASTER_DISPLAY_MODE_TFT,
                     RASTER_PALETTE_DATA, RASTER_COLOR, RASTER_RIGHT_ALIGNED);

    /* frame buffer data is ordered from least to Most significant bye */
    RasterLSBDataOrderSelect(SOC_LCDC_0_REGS);

    /* disable nibble mode */
    RasterNibbleModeDisable(SOC_LCDC_0_REGS);

     /* configuring the polarity of timing parameters of raster controller */
    RasterTiming2Configure(SOC_LCDC_0_REGS, RASTER_FRAME_CLOCK_LOW |
                                            RASTER_LINE_CLOCK_LOW  |
                                            RASTER_PIXEL_CLOCK_LOW |
                                            RASTER_SYNC_EDGE_RISING|
                                            RASTER_SYNC_CTRL_ACTIVE|
                                            RASTER_AC_BIAS_HIGH     , 0, 255);

    /* configuring horizontal timing parameter */
   RasterHparamConfig(SOC_LCDC_0_REGS, DISPLAY_IMAGE_WIDTH, 64, 48, 48);

    /* configuring vertical timing parameters */
   RasterVparamConfig(SOC_LCDC_0_REGS, DISPLAY_IMAGE_HEIGHT, 2, 11, 31);

    /* configuring fifo delay to */
    RasterFIFODMADelayConfig(SOC_LCDC_0_REGS, 2);
}

/*
** Configures arm/dsp interrupt controller to generate frame interrupt
*/
static void SetupIntc(void)
{
#ifdef _TMS320C6X
    /* Initialize the DSP interrupt controller */
    IntDSPINTCInit();

    /* Register VPIF ISR to vector table */
    IntRegister(C674X_MASK_INT5, VPIFIsr);

    /* Map system interrupt to DSP maskable interrupt for VPIF */
    IntEventMap(C674X_MASK_INT5, SYS_INT_VPIF_INT);

    /* Enable DSP maskable interrupt for VPIF */
    IntEnable(C674X_MASK_INT5);

    /* Register LCD ISR to vector table */
    IntRegister(C674X_MASK_INT6, LCDIsr);

    /* Map system interrupt to DSP maskable interrupt for LCD */
    IntEventMap(C674X_MASK_INT6, SYS_INT_LCDC_INT);

    /* Enable DSP maskable interrupt for LCD */
    IntEnable(C674X_MASK_INT6);

    /* Enable DSP interrupts */
    IntGlobalEnable();

#else
    /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_VPIF, VPIFIsr);

    /* Set the channel number 2 of AINTC for LCD system interrupt.  */
    IntChannelSet(SYS_INT_VPIF, 2);

    /* Enable the System Interrupts for AINTC.*/
    IntSystemEnable(SYS_INT_VPIF);

    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_LCDINT, LCDIsr);

    /* Set the channnel number 2 of AINTC for LCD system interrupt.  */
    IntChannelSet(SYS_INT_LCDINT, 3);

    /* Enable the System Interrupts for AINTC.*/
    IntSystemEnable(SYS_INT_LCDINT);

    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Enable the interrupts in GER of AINTC.*/
    IntGlobalEnable();

    /* Enable the interrupts in HIER of AINTC.*/
    IntIRQEnable();
#endif
}

/*
** VPIF Interrupt service routine.
*/
static void VPIFIsr(void)
{
#ifdef _TMS320C6X
    IntEventClear(SYS_INT_VPIF_INT);
#else
    IntSystemStatusClear(SYS_INT_VPIF);
#endif

    /* If previously captured frame not processed, clear this interrupt and return */
    if (!processed)
    {
        VPIFInterruptStatusClear(SOC_VPIF_0_REGS, VPIF_FRAMEINT_CH0);
        return;
    }

    /* buffcount represents buffer to be given to capture driver and
     * buffcount2 represents the newly captured buffer to be processed */
    processed = 0;
    captured = 0;
    buffcount++;
    buffcount2 = buffcount - 1;
    /* Currently only two buffers are being used for capture */
    if (buffcount == 2)
        buffcount = 0;

     /* Invalidate the buffers before giving to capture driver*/
#ifdef _TMS320C6X
    CacheInv((unsigned int) buff_luma[buffcount],
            CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT * 2);
    CacheInv((unsigned int) buff_chroma[buffcount],
            CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT * 2);
#else
    CP15ICacheFlushBuff((unsigned int) buff_luma[buffcount],
            CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT * 2);
    CP15ICacheFlushBuff((unsigned int) buff_chroma[buffcount],
            CAPTURE_IMAGE_WIDTH * CAPTURE_IMAGE_HEIGHT * 2);
#endif

    /* Initialize buffer addresses for a new frame*/
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_TOP_FIELD,
            VPIF_LUMA, (unsigned int) buff_luma[buffcount], CAPTURE_IMAGE_WIDTH*2);
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_TOP_FIELD,
            VPIF_CHROMA, (unsigned int) buff_chroma[buffcount], CAPTURE_IMAGE_WIDTH*2);
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_BOTTOM_FIELD,
            VPIF_LUMA, (unsigned int) (buff_luma[buffcount] + CAPTURE_IMAGE_WIDTH), CAPTURE_IMAGE_WIDTH*2);
    VPIFCaptureFBConfig(SOC_VPIF_0_REGS, VPIF_CHANNEL_0, VPIF_BOTTOM_FIELD,
            VPIF_CHROMA, (unsigned int) (buff_chroma[buffcount] + CAPTURE_IMAGE_WIDTH), CAPTURE_IMAGE_WIDTH*2);

    /* Initialize buffer addresses with the captured frame ready to be processed */
    videoTopC = buff_chroma[buffcount2];
    videoTopY = buff_luma[buffcount2];
    captured = 1;

    /* clear interrupt */
    VPIFInterruptStatusClear(SOC_VPIF_0_REGS, VPIF_FRAMEINT_CH0);
}

/*
** For each end of frame interrupt base and ceiling is reconfigured
*/
static void LCDIsr(void)
{
    unsigned int status;
#ifdef _TMS320C6X
    IntEventClear(SYS_INT_LCDC_INT);
#else
    IntSystemStatusClear(SYS_INT_LCDINT);
#endif

    /* Find which interrupt occurred and clear it */
    status = RasterIntStatus(SOC_LCDC_0_REGS,
            RASTER_END_OF_FRAME0_INT_STAT | RASTER_END_OF_FRAME1_INT_STAT);
    status = RasterClearGetIntStatus(SOC_LCDC_0_REGS, status);

    /* Display the appropriate output buffer on the appropriate raster buffer
     * and if a new processed buffer is available, let the DSP know that
     * it has configured the raster buffer to point to the new output buffer
     * by updating the 'updated' flag */
    if (display_buff_1)
    {
        if (status & RASTER_END_OF_FRAME0_INT_STAT)
        {
            RasterDMAFBConfig(SOC_LCDC_0_REGS, (unsigned int) Rgb_buffer1,
                    (unsigned int) (Rgb_buffer1 + DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT + 15), 0);
            if (changed)
                updated = updated | 0x1;
        }
        if (status & RASTER_END_OF_FRAME1_INT_STAT)
        {
            RasterDMAFBConfig(SOC_LCDC_0_REGS, (unsigned int) Rgb_buffer1,
                    (unsigned int) (Rgb_buffer1 + DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT + 15), 1);
            if (changed)
                updated = updated | 0x2;
        }
    }
    else
    {
        if (status & RASTER_END_OF_FRAME0_INT_STAT)
        {
            RasterDMAFBConfig(SOC_LCDC_0_REGS, (unsigned int) Rgb_buffer2,
                    (unsigned int) (Rgb_buffer2 + DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT + 15), 0);
            if (changed)
                updated = updated | 0x1;
        }
        if (status & RASTER_END_OF_FRAME1_INT_STAT)
        {
            RasterDMAFBConfig(SOC_LCDC_0_REGS, (unsigned int) Rgb_buffer2,
                    (unsigned int) (Rgb_buffer2 + DISPLAY_IMAGE_WIDTH * DISPLAY_IMAGE_HEIGHT + 15), 1);
            if (changed)
                updated = updated | 0x2;
        }
    }
}

/***************************** End Of File ************************************/
