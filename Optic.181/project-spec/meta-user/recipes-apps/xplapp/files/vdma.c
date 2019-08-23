#include <stdio.h>
#include <string.h>
#include "xilinx/xiltypes.h"
#include "xplioctl.h"
#include "debug.h"
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>


#define RVDMA1_BASE_MEM				0x50000000			// Image out(1920*1080)
#define WVDMA5_BASE_MEM				0x50000000			// Imge In(1536 x 864)

#define WVDMA4_BASE_MEM				0x43000000			// Image In(1920x1080)
#define RVDMA9_BASE_MEM				0x43000000
#define RVDMA7_BASE_MEM				0x43000000

#define WVDMA6_BASE_MEM				0x60000000			// Image In(720x480)
#define RVDMA8_BASE_MEM				0x60000000

XAxiVdma rInstancePtr1;
XAxiVdma rInstancePtr3;
XAxiVdma wInstancePtr4;
XAxiVdma wInstancePtr5;
XAxiVdma wInstancePtr6;
XAxiVdma rInstancePtr7;
XAxiVdma rInstancePtr8;
XAxiVdma rInstancePtr9;

static XAxiVdma InstancePtr4;
static XAxiVdma_DmaSetup ReadCfg4;

XAxiVdma_DmaSetup rReadCfg1;
XAxiVdma_DmaSetup rReadCfg3;
XAxiVdma_DmaSetup wReadCfg4;
XAxiVdma_DmaSetup wReadCfg5;
XAxiVdma_DmaSetup wReadCfg6;
XAxiVdma_DmaSetup rReadCfg7;
XAxiVdma_DmaSetup rReadCfg8;
XAxiVdma_DmaSetup rReadCfg9;

#define VDMA1_FRAME_DEVICE_ID		0
#define VDMA3_FRAME_DEVICE_ID		2
#define VDMA4_FRAME_DEVICE_ID		1
#define VDMA5_FRAME_DEVICE_ID		3
#define VDMA6_FRAME_DEVICE_ID		4
#define VDMA7_FRAME_DEVICE_ID		5
#define VDMA8_FRAME_DEVICE_ID		6
#define VDMA9_FRAME_DEVICE_ID		7

uint32_t rfb1_mem;
uint32_t wfb4_mem;
uint32_t wfb5_mem;
uint32_t wfb6_mem;
uint32_t rfb7_mem;
uint32_t rfb8_mem;
uint32_t rfb9_mem;

#define XVTC1_DEVICE_ID				0
#define XVTC2_DEVICE_ID				1
#define XVTC5_DEVICE_ID				3
#define XVTC4_DEVICE_ID				2

uint32_t vtc1_mem;
uint32_t vtc2_mem;
uint32_t vtc5_mem;
uint32_t vtc4_mem;


#define SCALER1_DEVICE_ID			0
#define SCALER2_DEVICE_ID			1


static void XAxiVdma_BdSetNextPtr(XAxiVdma_Bd *BdPtr, u32 NextPtr);
static void XAxiVdma_BdWrite(XAxiVdma_Bd *BdPtr, int Offset, u32 Value);
static int XAxiVdma_BdSetVsize(XAxiVdma_Bd *BdPtr, int Vsize);
static int XAxiVdma_BdSetHsize(XAxiVdma_Bd *BdPtr, int Hsize);
static int XAxiVdma_BdSetStride(XAxiVdma_Bd *BdPtr, int Stride);
static int XAxiVdma_BdSetFrmDly(XAxiVdma_Bd *BdPtr, int FrmDly);
static void XAxiVdma_BdSetAddr(XAxiVdma_Bd *BdPtr, u32 Addr);


static uint32_t *vdmaAddr;
static int      vdmafd;
static size_t vdmalength = 0x100000;
static uint32_t vdma_orgBase;

static uint32_t *vtcAddr;
static int      vtcfd;
static size_t vtclength = 0x100000;

//--------------------------------------------------------------------------------------------------------------
void Vdma_Open(uint32_t addr)
{
	vdmafd = open("/dev/mem", O_RDWR | O_SYNC);
  	if (vdmafd < 0) {
      	fprintf(stderr, "%s", strerror(errno));
  	}

  	vdmaAddr = mmap(NULL, vdmalength, PROT_READ | PROT_WRITE, MAP_SHARED, vdmafd, addr);

  	// memset(vdmaAddr, 0x00, vdmalength / 4);

  	xil_printf("[VDMA OK] : VDMA Open %x\n", vdmafd);
}
//--------------------------------------------------------------------------------------------------------------
void Vdma_Close()
{
  	munmap(vdmaAddr, vdmalength);
  	close(vdmafd);
  	xil_printf("[VDMA OK] : VDMA close\n");
}
//--------------------------------------------------------------------------------------------------------------
void Vtc_Open(uint32_t addr)
{
	vtcfd = open("/dev/mem", O_RDWR | O_SYNC);
  	if (vtcfd < 0) {
      	fprintf(stderr, "%s", strerror(errno));
  	}

  	vtcAddr = mmap(NULL, vtclength, PROT_READ | PROT_WRITE, MAP_SHARED, vtcfd, addr);
  	// memset(vtcAddr, 0x00, vtclength / 4);
  	xil_printf("[VDMA OK] : VTC Open %x\n", vdmafd);
}
//--------------------------------------------------------------------------------------------------------------
void Vtc_Close()
{
  	munmap(vtcAddr, vtclength);
  	close(vtcfd);
  	xil_printf("[VTC OK] : VTC close\n");
}
//--------------------------------------------------------------------------------------------------------------
uint32_t XAxiVdma_ReadReg(UINTPTR base, int offset)
{
	/*
    uint32_t regv = 0;
    uint32_t _offset = (uint32_t)base + offset;
    if (reg_read32(_offset, &regv) != IOK) {
        ERR("READ32(offset:%08X)", _offset);
    }
    DBG("RD32:%08X -> %08X\n", _offset, regv);
    return regv;*/

    uint32_t regv = 0;

    uint32_t _offset = (uint32_t)vdmaAddr + (((unsigned int)(base - vdma_orgBase)/4) + (offset/4));

    regv  = *(vdmaAddr + ((unsigned int)(base-vdma_orgBase)/4) + ((unsigned int)offset/4) );
    // DBG("RD32:%08X -> %08X\n", _offset, regv);
    return regv;
}
//--------------------------------------------------------------------------------------------------------------
void XAxiVdma_WriteReg(UINTPTR base, int offset, uint32_t data)
{
	/*
    uint32_t _offset = (uint32_t)base + offset;
    DBG("WR32:%08X <- %08X\n", _offset, data);
    if (reg_write32(_offset, data) != IOK) {
        ERR("WRITE32(offset:%08X, %08X)", _offset, data);
    }*/

    uint32_t _offset = 0;
    _offset = (uint32_t)vdmaAddr + (((unsigned int)(base - vdma_orgBase)/4) + (offset/4));

    *(vdmaAddr + ((unsigned int)(base-vdma_orgBase)/4) + ((unsigned int )offset/4)) = data;
    // DBG("WR32:%08X <- %08X(%08X:%08X)\n", _offset, data, base, offset);
}
//--------------------------------------------------------------------------------------------------------------
uint32_t XVtc_ReadReg(UINTPTR base, int offset)
{
	/*
    uint32_t regv = 0;
    uint32_t _offset = (uint32_t)base + offset;
    if (reg_read32(_offset, &regv) != IOK) {
        ERR("READ32(offset:%08X)", _offset);
    }
    DBG("RD32:%08X -> %08X\n", _offset, regv);
    return regv;*/

    uint32_t regv = 0;

    uint32_t _offset = (uint32_t)vtcAddr + (offset/4);
    regv  = *(vtcAddr + ((unsigned int)offset/4));

    // DBG("VTC RD32:%08X -> %08X\n", _offset, regv);
    return regv;
}
//--------------------------------------------------------------------------------------------------------------
void XVtc_WriteReg(UINTPTR base, int offset, uint32_t data)
{
	/*
    uint32_t _offset = (uint32_t)base + offset;
    DBG("WR32:%08X <- %08X\n", _offset, data);
    if (reg_write32(_offset, data) != IOK) {
        ERR("WRITE32(offset:%08X, %08X)", _offset, data);
    }*/

    uint32_t _offset = 0;
    _offset = (uint32_t)vtcAddr + (offset/4);

    *(vtcAddr + ((unsigned int )offset/4)) = data;

    // DBG("VTC WR32:%08X <- %08X\n", _offset, data);
}

//--------------------------------------------------------------------------------------------------------------
#define XAXIVDMA_VIRT_TO_PHYS(VirtAddr) \
    (VirtAddr)
//--------------------------------------------------------------------------------------------------------------
#define XAxiVdma_ChannelHiFrmAddrEnable(Channel) \
{ \
    XAxiVdma_WriteReg(Channel->ChanBase, \
            XAXIVDMA_HI_FRMBUF_OFFSET, XAXIVDMA_REGINDEX_MASK); \
}
//--------------------------------------------------------------------------------------------------------------
#define XAxiVdma_ChannelHiFrmAddrDisable(Channel) \
{ \
    XAxiVdma_WriteReg(Channel->ChanBase, \
        XAXIVDMA_HI_FRMBUF_OFFSET, (XAXIVDMA_REGINDEX_MASK >> 1)); \
}
//--------------------------------------------------------------------------------------------------------------
 void XAxiVdma_ChannelInit(XAxiVdma_Channel *Channel)
 {
     int i;
     int NumFrames;
     XAxiVdma_Bd *FirstBdPtr = &(Channel->BDs[0]);
     XAxiVdma_Bd *LastBdPtr;

     // Initialize the BD variables, so proper memory management can be done
     NumFrames = Channel->NumFrames;
     Channel->IsValid = 0;
     Channel->HeadBdPhysAddr = 0;
     Channel->HeadBdAddr = 0;
     Channel->TailBdPhysAddr = 0;
     Channel->TailBdAddr = 0;

     LastBdPtr = &(Channel->BDs[NumFrames - 1]);

     // Setup the BD ring
     memset((void *)FirstBdPtr, 0, NumFrames * sizeof(XAxiVdma_Bd));

     for (i = 0; i < NumFrames; i++) {
         XAxiVdma_Bd *BdPtr;
         XAxiVdma_Bd *NextBdPtr;

         BdPtr = &(Channel->BDs[i]);

         // The last BD connects to the first BD
         if (i == (NumFrames - 1)) {
             NextBdPtr = FirstBdPtr;
         }
         else {
             NextBdPtr = &(Channel->BDs[i + 1]);
         }

         XAxiVdma_BdSetNextPtr(BdPtr,
                 XAXIVDMA_VIRT_TO_PHYS((UINTPTR)NextBdPtr));
     }

     Channel->AllCnt = NumFrames;

     // Setup the BD addresses so that access the head/tail BDs fast
     Channel->HeadBdAddr = (UINTPTR)FirstBdPtr;
     Channel->HeadBdPhysAddr = XAXIVDMA_VIRT_TO_PHYS((UINTPTR)FirstBdPtr);

     Channel->TailBdAddr = (UINTPTR)LastBdPtr;
     Channel->TailBdPhysAddr = XAXIVDMA_VIRT_TO_PHYS((UINTPTR)LastBdPtr);


     Channel->IsValid = 1;

     return;
 }
//--------------------------------------------------------------------------------------------------------------
 int XAxiVdma_ChannelResetNotDone(XAxiVdma_Channel *Channel)
 {
     return (XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
             XAXIVDMA_CR_RESET_MASK);
 }
//--------------------------------------------------------------------------------------------------------------
 void XAxiVdma_ChannelReset(XAxiVdma_Channel *Channel)
 {
     XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET, XAXIVDMA_CR_RESET_MASK);

     return;
 }
//--------------------------------------------------------------------------------------------------------------
 int XAxiVdma_ChannelIsRunning(XAxiVdma_Channel *Channel)
 {
     u32 Bits;

     // If halted bit set, channel is not running
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
               XAXIVDMA_SR_HALTED_MASK;

     if (Bits) {
         return 0;
     }

     // If Run/Stop bit low, then channel is not running
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
               XAXIVDMA_CR_RUNSTOP_MASK;

     if (!Bits) {
         return 0;
     }

     return 1;
 }
//--------------------------------------------------------------------------------------------------------------
 int XAxiVdma_ChannelIsBusy(XAxiVdma_Channel *Channel)
 {
     u32 Bits;

     // If the channel is idle, then it is not busy
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
               XAXIVDMA_SR_IDLE_MASK;

     if (Bits) {
         return 0;
     }

     // If the channel is halted, then it is not busy
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
               XAXIVDMA_SR_HALTED_MASK;

     if (Bits) {
         return 0;
     }

     // Otherwise, it is busy
     return 1;
 }
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_ChannelConfig(XAxiVdma_Channel *Channel,
       XAxiVdma_ChannelSetup *ChannelCfgPtr)
{
   u32 CrBits;
   int i;
   int NumBds;
   int Status;
   u32 hsize_align;
   u32 stride_align;

   if (!Channel->IsValid) {
       xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");

       return XST_FAILURE;
   }

   if (Channel->HasSG && XAxiVdma_ChannelIsBusy(Channel)) {
       xdbg_printf(XDBG_DEBUG_ERROR,
           "Channel is busy, cannot config!\r\n");

       return XST_DEVICE_BUSY;
   }

   Channel->Vsize = ChannelCfgPtr->VertSizeInput;

   // Check whether Hsize is properly aligned
   if (Channel->direction == XAXIVDMA_WRITE) {
       if (ChannelCfgPtr->HoriSizeInput < Channel->WordLength) {
           hsize_align = (u32)Channel->WordLength;
       } else {
           hsize_align =
               (u32)(ChannelCfgPtr->HoriSizeInput % Channel->WordLength);
           if (hsize_align > 0)
               hsize_align = (Channel->WordLength - hsize_align);
       }
   } else {
       if (ChannelCfgPtr->HoriSizeInput < Channel->WordLength) {
           hsize_align = (u32)Channel->WordLength;
       } else {
           hsize_align =
               (u32)(ChannelCfgPtr->HoriSizeInput % Channel->StreamWidth);
           if (hsize_align > 0)
               hsize_align = (Channel->StreamWidth - hsize_align);
       }
   }

   // Check whether Stride is properly aligned
   if (ChannelCfgPtr->Stride < Channel->WordLength) {
       stride_align = (u32)Channel->WordLength;
   } else {
       stride_align = (u32)(ChannelCfgPtr->Stride % Channel->WordLength);
       if (stride_align > 0)
           stride_align = (Channel->WordLength - stride_align);
   }
   // If hardware has no DRE, then Hsize and Stride must be word-aligned
   if (!Channel->HasDRE) {
       if (hsize_align != 0) {
           // Adjust hsize to multiples of stream/mm data width
           ChannelCfgPtr->HoriSizeInput += hsize_align;
       }
       if (stride_align != 0) {
           // Adjust stride to multiples of stream/mm data width
           ChannelCfgPtr->Stride += stride_align;
       }
   }

   Channel->Hsize = ChannelCfgPtr->HoriSizeInput;

   CrBits = XAxiVdma_ReadReg(Channel->ChanBase,
        XAXIVDMA_CR_OFFSET);

   CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
       ~(XAXIVDMA_CR_TAIL_EN_MASK | XAXIVDMA_CR_SYNC_EN_MASK |
         XAXIVDMA_CR_FRMCNT_EN_MASK | XAXIVDMA_CR_RD_PTR_MASK);

   if (ChannelCfgPtr->EnableCircularBuf) {
       CrBits |= XAXIVDMA_CR_TAIL_EN_MASK;
   }
   else {
       // Park mode
       u32 FrmBits;
       u32 RegValue;

       if ((!XAxiVdma_ChannelIsRunning(Channel)) &&
           Channel->HasSG) {
           xdbg_printf(XDBG_DEBUG_ERROR,
               "Channel is not running, cannot set park mode\r\n");

           return XST_INVALID_PARAM;
       }

       if (ChannelCfgPtr->FixedFrameStoreAddr > XAXIVDMA_FRM_MAX) {
           xdbg_printf(XDBG_DEBUG_ERROR,
               "Invalid frame to park on %d\r\n",
               ChannelCfgPtr->FixedFrameStoreAddr);

           return XST_INVALID_PARAM;
       }
       if (ChannelCfgPtr->FixedFrameStoreAddr > XAXIVDMA_FRM_MAX) {
           xdbg_printf(XDBG_DEBUG_ERROR,
               "Invalid frame to park on %d\r\n",
               ChannelCfgPtr->FixedFrameStoreAddr);

           return XST_INVALID_PARAM;
       }

       if (Channel->IsRead) {
           FrmBits = ChannelCfgPtr->FixedFrameStoreAddr &
                         XAXIVDMA_PARKPTR_READREF_MASK;

           RegValue = XAxiVdma_ReadReg(Channel->InstanceBase,
                         XAXIVDMA_PARKPTR_OFFSET);

           RegValue &= ~XAXIVDMA_PARKPTR_READREF_MASK;

           RegValue |= FrmBits;

           XAxiVdma_WriteReg(Channel->InstanceBase,
               XAXIVDMA_PARKPTR_OFFSET, RegValue);
       }
       else {
           FrmBits = ChannelCfgPtr->FixedFrameStoreAddr <<
                       XAXIVDMA_WRTREF_SHIFT;

           FrmBits &= XAXIVDMA_PARKPTR_WRTREF_MASK;

           RegValue = XAxiVdma_ReadReg(Channel->InstanceBase,
                         XAXIVDMA_PARKPTR_OFFSET);

           RegValue &= ~XAXIVDMA_PARKPTR_WRTREF_MASK;

           RegValue |= FrmBits;

           XAxiVdma_WriteReg(Channel->InstanceBase,
               XAXIVDMA_PARKPTR_OFFSET, RegValue);
       }
   }

   if (ChannelCfgPtr->EnableSync) {
       if (Channel->GenLock != XAXIVDMA_GENLOCK_MASTER)
           CrBits |= XAXIVDMA_CR_SYNC_EN_MASK;
   }

   if (ChannelCfgPtr->GenLockRepeat) {
       if ((Channel->GenLock == XAXIVDMA_GENLOCK_MASTER) ||
           (Channel->GenLock == XAXIVDMA_DYN_GENLOCK_MASTER))
           CrBits |= XAXIVDMA_CR_GENLCK_RPT_MASK;
   }
   if (ChannelCfgPtr->EnableFrameCounter) {
       CrBits |= XAXIVDMA_CR_FRMCNT_EN_MASK;
   }

   CrBits |= (ChannelCfgPtr->PointNum << XAXIVDMA_CR_RD_PTR_SHIFT) &
       XAXIVDMA_CR_RD_PTR_MASK;

   // Write the control register value out
   XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
       CrBits);

   if (Channel->HasSG) {
       // Setup the information in BDs
       NumBds = Channel->AllCnt;

       for (i = 0; i < NumBds; i++) {
           XAxiVdma_Bd *BdPtr = (XAxiVdma_Bd *)(Channel->HeadBdAddr +
                    i * sizeof(XAxiVdma_Bd));

           Status = XAxiVdma_BdSetVsize(BdPtr,
                        ChannelCfgPtr->VertSizeInput);
           if (Status != XST_SUCCESS) {
               xdbg_printf(XDBG_DEBUG_ERROR,
                   "Set vertical size failed %d\r\n", Status);

               return Status;
           }

           Status = XAxiVdma_BdSetHsize(BdPtr,
               ChannelCfgPtr->HoriSizeInput);
           if (Status != XST_SUCCESS) {
               xdbg_printf(XDBG_DEBUG_ERROR,
                   "Set horizontal size failed %d\r\n", Status);

               return Status;
           }

           Status = XAxiVdma_BdSetStride(BdPtr,
               ChannelCfgPtr->Stride);
           if (Status != XST_SUCCESS) {
               xdbg_printf(XDBG_DEBUG_ERROR,
                   "Set stride size failed %d\r\n", Status);

               return Status;
           }

           Status = XAxiVdma_BdSetFrmDly(BdPtr,
           ChannelCfgPtr->FrameDelay);
           if (Status != XST_SUCCESS) {
               xdbg_printf(XDBG_DEBUG_ERROR,
                   "Set frame delay failed %d\r\n", Status);

               return Status;
           }
       }
   }
   else {   // direct register mode */
       if ((ChannelCfgPtr->VertSizeInput > XAXIVDMA_MAX_VSIZE) ||
           (ChannelCfgPtr->VertSizeInput <= 0) ||
           (ChannelCfgPtr->HoriSizeInput > XAXIVDMA_MAX_HSIZE) ||
           (ChannelCfgPtr->HoriSizeInput <= 0) ||
           (ChannelCfgPtr->Stride > XAXIVDMA_MAX_STRIDE) ||
           (ChannelCfgPtr->Stride <= 0) ||
           (ChannelCfgPtr->FrameDelay < 0) ||
           (ChannelCfgPtr->FrameDelay > XAXIVDMA_FRMDLY_MAX)) {

           return XST_INVALID_PARAM;
       }

       XAxiVdma_WriteReg(Channel->StartAddrBase,
           XAXIVDMA_HSIZE_OFFSET, ChannelCfgPtr->HoriSizeInput);

       XAxiVdma_WriteReg(Channel->StartAddrBase,
           XAXIVDMA_STRD_FRMDLY_OFFSET,
           (ChannelCfgPtr->FrameDelay << XAXIVDMA_FRMDLY_SHIFT) |
           ChannelCfgPtr->Stride);
   }

   return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_ChannelSetBufferAddr(XAxiVdma_Channel *Channel,
        UINTPTR *BufferAddrSet, int NumFrames)
{
    int i;
    u32 WordLenBits;
    int HiFrmAddr = 0;
    int FrmBound;
    if (Channel->AddrWidth > 32) {
        FrmBound = (XAXIVDMA_MAX_FRAMESTORE_64)/2 - 1;
    } else {
        FrmBound = (XAXIVDMA_MAX_FRAMESTORE)/2 - 1;
    }
    int Loop16 = 0;
  printf("@ %4d:%s NumFrames=%d\n",__LINE__, __FUNCTION__, NumFrames);
  printf("@ %4d:%s Addr=%X\n",__LINE__, __FUNCTION__, BufferAddrSet[0]);

    if (!Channel->IsValid) {
        xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");

        return XST_FAILURE;
    }

    WordLenBits = (u32)(Channel->WordLength - 1);

    // If hardware has no DRE, then buffer addresses must be word-aligned
    for (i = 0; i < NumFrames; i++) {
        if (!Channel->HasDRE) {
            if (BufferAddrSet[i] & WordLenBits) {
                xdbg_printf(XDBG_DEBUG_ERROR,
                    "Unaligned address %d: %x without DRE\r\n",
                    i, BufferAddrSet[i]);

                return XST_INVALID_PARAM;
            }
        }
    }
    for (i = 0; i < NumFrames; i++, Loop16++) {
        XAxiVdma_Bd *BdPtr = (XAxiVdma_Bd *)(Channel->HeadBdAddr +
                 i * sizeof(XAxiVdma_Bd));

        if (Channel->HasSG) {
            XAxiVdma_BdSetAddr(BdPtr, BufferAddrSet[i]);
        }
        else {
            if ((i > FrmBound) && !HiFrmAddr) {
                XAxiVdma_ChannelHiFrmAddrEnable(Channel);
                HiFrmAddr = 1;
                Loop16 = 0;
            }

            if (Channel->AddrWidth > 32) {
                // For a 40-bit address XAXIVDMA_MAX_FRAMESTORE value should be set to 16
                XAxiVdma_WriteReg(Channel->StartAddrBase,
                    XAXIVDMA_START_ADDR_OFFSET +
                    Loop16 * XAXIVDMA_START_ADDR_LEN + i*4,
                    LOWER_32_BITS(BufferAddrSet[i]));

//              printf("@ %4d:%s\n",__LINE__, __FUNCTION__);
                XAxiVdma_WriteReg(Channel->StartAddrBase,
                    XAXIVDMA_START_ADDR_MSB_OFFSET +
                    Loop16 * XAXIVDMA_START_ADDR_LEN + i*4,
                    UPPER_32_BITS((u64)BufferAddrSet[i]));
            } else {
//              printf("@ %4d:%s\n",__LINE__, __FUNCTION__);
                XAxiVdma_WriteReg(Channel->StartAddrBase,
                    XAXIVDMA_START_ADDR_OFFSET +
                    Loop16 * XAXIVDMA_START_ADDR_LEN,
                    BufferAddrSet[i]);
            }


            if ((NumFrames > FrmBound) && (i == (NumFrames - 1)))
                XAxiVdma_ChannelHiFrmAddrDisable(Channel);
        }
    }

    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_ChannelStart(XAxiVdma_Channel *Channel)
{
    u32 CrBits;

    if (!Channel->IsValid) {
        xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");

        return XST_FAILURE;
    }

    if (Channel->HasSG && XAxiVdma_ChannelIsBusy(Channel)) {

        xdbg_printf(XDBG_DEBUG_ERROR,
            "Start DMA channel while channel is busy\r\n");

        return XST_DEVICE_BUSY;
    }else
		xil_printf("Channel initialized\r\n");

    // If channel is not running, setup the CDESC register and set the channel to run
    if (!XAxiVdma_ChannelIsRunning(Channel)) {

        if (Channel->HasSG) {
            // Set up the current bd register
            // Can only setup current bd register when channel is halted
            XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CDESC_OFFSET,
                Channel->HeadBdPhysAddr);
        }

        // Start DMA hardware
        CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET);

        CrBits = XAxiVdma_ReadReg(Channel->ChanBase,
             XAXIVDMA_CR_OFFSET) | XAXIVDMA_CR_RUNSTOP_MASK;

        XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
            CrBits);

    }

    if (XAxiVdma_ChannelIsRunning(Channel)) {

        // Start DMA transfers

        if (Channel->HasSG) {
            // SG mode:
            XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_TDESC_OFFSET,
               Channel->TailBdPhysAddr);
        }
        else {
            // Direct register mode:
            XAxiVdma_WriteReg(Channel->StartAddrBase,
                XAXIVDMA_VSIZE_OFFSET, Channel->Vsize);

        }

        return XST_SUCCESS;
    }
    else {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "Failed to start channel %x\r\n", (unsigned int)Channel->ChanBase);

        return XST_DMA_ERROR;
    }
}
//--------------------------------------------------------------------------------------------------------------
void XAxiVdma_ChannelStop(XAxiVdma_Channel *Channel)
{
    u32 CrBits;

    if (!XAxiVdma_ChannelIsRunning(Channel)) {
        return;
    }

    // Clear the RS bit in CR register
    CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
        (~XAXIVDMA_CR_RUNSTOP_MASK);

    XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET, CrBits);

    return;
}
//--------------------------------------------------------------------------------------------------------------
void XAxiVdma_ChannelRegisterDump(XAxiVdma_Channel *Channel)
{
    xil_printf("Dump register for channel %p:\r\n", (void *)Channel->ChanBase);
    xil_printf("\tControl Reg: %x\r\n",
        XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET));
    xil_printf("\tStatus Reg: %x\r\n",
        XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET));
    xil_printf("\tCDESC Reg: %x\r\n",
        XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CDESC_OFFSET));
    xil_printf("\tTDESC Reg: %x\r\n",
        XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_TDESC_OFFSET));

    return;
}
//--------------------------------------------------------------------------------------------------------------
static u32 XAxiVdma_BdRead(XAxiVdma_Bd *BdPtr, int Offset)
{
    DBG("%s: RD %08X\n", __FUNCTION__, (uint32_t)BdPtr+Offset);
    return (*(u32 *)((UINTPTR)(void *)BdPtr + Offset));
}
//--------------------------------------------------------------------------------------------------------------
static void XAxiVdma_BdWrite(XAxiVdma_Bd *BdPtr, int Offset, u32 Value)
{
    // DBG("%s: WR %08X <- %08X\n", __FUNCTION__, (uint32_t)BdPtr+Offset, Value);
    // *(u32 *)((UINTPTR)(void *)BdPtr + Offset) = Value;

    return;
}
//--------------------------------------------------------------------------------------------------------------
static void XAxiVdma_BdSetNextPtr(XAxiVdma_Bd *BdPtr, u32 NextPtr)
{
    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_NDESC_OFFSET, NextPtr);
    return;
}
//--------------------------------------------------------------------------------------------------------------
static void XAxiVdma_BdSetAddr(XAxiVdma_Bd *BdPtr, u32 Addr)
{
    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_START_ADDR_OFFSET, Addr);

    return;
}
//--------------------------------------------------------------------------------------------------------------
static int XAxiVdma_BdSetVsize(XAxiVdma_Bd *BdPtr, int Vsize)
{
    if ((Vsize <= 0) || (Vsize > XAXIVDMA_VSIZE_MASK)) {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "Veritcal size %d is not valid\r\n", Vsize);

        return XST_INVALID_PARAM;
    }

    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_VSIZE_OFFSET, Vsize);
    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
static int XAxiVdma_BdSetHsize(XAxiVdma_Bd *BdPtr, int Hsize)
{
    if ((Hsize <= 0) || (Hsize > XAXIVDMA_HSIZE_MASK)) {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "Horizontal size %d is not valid\r\n", Hsize);

        return XST_INVALID_PARAM;
    }

    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_HSIZE_OFFSET, Hsize);
    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
static int XAxiVdma_BdSetStride(XAxiVdma_Bd *BdPtr, int Stride)
{
    u32 Bits;

    if ((Stride <= 0) || (Stride > XAXIVDMA_STRIDE_MASK)) {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "Stride size %d is not valid\r\n", Stride);

        return XST_INVALID_PARAM;
    }

    Bits = XAxiVdma_BdRead(BdPtr, XAXIVDMA_BD_STRIDE_OFFSET) &
            ~XAXIVDMA_STRIDE_MASK;

    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_STRIDE_OFFSET, Bits | Stride);

    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
static int XAxiVdma_BdSetFrmDly(XAxiVdma_Bd *BdPtr, int FrmDly)
{
    u32 Bits;

    if ((FrmDly < 0) || (FrmDly > XAXIVDMA_FRMDLY_MAX)) {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "FrmDly size %d is not valid\r\n", FrmDly);

        return XST_INVALID_PARAM;
    }

    Bits = XAxiVdma_BdRead(BdPtr, XAXIVDMA_BD_STRIDE_OFFSET) &
            ~XAXIVDMA_FRMDLY_MASK;

    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_STRIDE_OFFSET,
        Bits | (FrmDly << XAXIVDMA_FRMDLY_SHIFT));

    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
XAxiVdma_Config XAxiVdma_ConfigTable[XPAR_XAXIVDMA_NUM_INSTANCES] =
{
	{
		XPAR_AXI_VDMA_1_DEVICE_ID,
		XPAR_AXI_VDMA_1_BASEADDR,
		XPAR_AXI_VDMA_1_NUM_FSTORES,
		XPAR_AXI_VDMA_1_INCLUDE_MM2S,
		XPAR_AXI_VDMA_1_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_1_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_1_INCLUDE_S2MM,
		XPAR_AXI_VDMA_1_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_1_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_1_INCLUDE_SG,
		XPAR_AXI_VDMA_1_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_1_USE_FSYNC,
		XPAR_AXI_VDMA_1_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_1_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_1_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_1_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_1_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_1_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_1_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_1_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_1_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_1_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_1_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_4_DEVICE_ID,
		XPAR_AXI_VDMA_4_BASEADDR,
		XPAR_AXI_VDMA_4_NUM_FSTORES,
		XPAR_AXI_VDMA_4_INCLUDE_MM2S,
		XPAR_AXI_VDMA_4_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_4_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_4_INCLUDE_S2MM,
		XPAR_AXI_VDMA_4_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_4_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_4_INCLUDE_SG,
		XPAR_AXI_VDMA_4_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_4_USE_FSYNC,
		XPAR_AXI_VDMA_4_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_4_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_4_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_4_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_4_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_4_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_4_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_4_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_4_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_4_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_4_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_3_DEVICE_ID,
		XPAR_AXI_VDMA_3_BASEADDR,
		XPAR_AXI_VDMA_3_NUM_FSTORES,
		XPAR_AXI_VDMA_3_INCLUDE_MM2S,
		XPAR_AXI_VDMA_3_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_3_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_3_INCLUDE_S2MM,
		XPAR_AXI_VDMA_3_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_3_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_3_INCLUDE_SG,
		XPAR_AXI_VDMA_3_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_3_USE_FSYNC,
		XPAR_AXI_VDMA_3_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_3_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_3_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_3_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_3_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_3_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_3_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_3_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_3_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_3_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_3_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_5_DEVICE_ID,
		XPAR_AXI_VDMA_5_BASEADDR,
		XPAR_AXI_VDMA_5_NUM_FSTORES,
		XPAR_AXI_VDMA_5_INCLUDE_MM2S,
		XPAR_AXI_VDMA_5_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_5_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_5_INCLUDE_S2MM,
		XPAR_AXI_VDMA_5_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_5_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_5_INCLUDE_SG,
		XPAR_AXI_VDMA_5_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_5_USE_FSYNC,
		XPAR_AXI_VDMA_5_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_5_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_5_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_5_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_5_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_5_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_5_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_5_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_5_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_5_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_5_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_6_DEVICE_ID,
		XPAR_AXI_VDMA_6_BASEADDR,
		XPAR_AXI_VDMA_6_NUM_FSTORES,
		XPAR_AXI_VDMA_6_INCLUDE_MM2S,
		XPAR_AXI_VDMA_6_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_6_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_6_INCLUDE_S2MM,
		XPAR_AXI_VDMA_6_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_6_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_6_INCLUDE_SG,
		XPAR_AXI_VDMA_6_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_6_USE_FSYNC,
		XPAR_AXI_VDMA_6_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_6_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_6_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_6_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_6_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_6_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_6_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_6_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_6_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_6_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_6_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_7_DEVICE_ID,
		XPAR_AXI_VDMA_7_BASEADDR,
		XPAR_AXI_VDMA_7_NUM_FSTORES,
		XPAR_AXI_VDMA_7_INCLUDE_MM2S,
		XPAR_AXI_VDMA_7_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_7_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_7_INCLUDE_S2MM,
		XPAR_AXI_VDMA_7_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_7_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_7_INCLUDE_SG,
		XPAR_AXI_VDMA_7_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_7_USE_FSYNC,
		XPAR_AXI_VDMA_7_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_7_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_7_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_7_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_7_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_7_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_7_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_7_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_7_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_7_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_7_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_8_DEVICE_ID,
		XPAR_AXI_VDMA_8_BASEADDR,
		XPAR_AXI_VDMA_8_NUM_FSTORES,
		XPAR_AXI_VDMA_8_INCLUDE_MM2S,
		XPAR_AXI_VDMA_8_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_8_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_8_INCLUDE_S2MM,
		XPAR_AXI_VDMA_8_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_8_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_8_INCLUDE_SG,
		XPAR_AXI_VDMA_8_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_8_USE_FSYNC,
		XPAR_AXI_VDMA_8_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_8_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_8_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_8_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_8_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_8_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_8_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_8_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_8_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_8_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_8_ADDR_WIDTH
	},
	{
		XPAR_AXI_VDMA_9_DEVICE_ID,
		XPAR_AXI_VDMA_9_BASEADDR,
		XPAR_AXI_VDMA_9_NUM_FSTORES,
		XPAR_AXI_VDMA_9_INCLUDE_MM2S,
		XPAR_AXI_VDMA_9_INCLUDE_MM2S_DRE,
		XPAR_AXI_VDMA_9_M_AXI_MM2S_DATA_WIDTH,
		XPAR_AXI_VDMA_9_INCLUDE_S2MM,
		XPAR_AXI_VDMA_9_INCLUDE_S2MM_DRE,
		XPAR_AXI_VDMA_9_M_AXI_S2MM_DATA_WIDTH,
		XPAR_AXI_VDMA_9_INCLUDE_SG,
		XPAR_AXI_VDMA_9_ENABLE_VIDPRMTR_READS,
		XPAR_AXI_VDMA_9_USE_FSYNC,
		XPAR_AXI_VDMA_9_FLUSH_ON_FSYNC,
		XPAR_AXI_VDMA_9_MM2S_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_9_S2MM_LINEBUFFER_DEPTH,
		XPAR_AXI_VDMA_9_MM2S_GENLOCK_MODE,
		XPAR_AXI_VDMA_9_S2MM_GENLOCK_MODE,
		XPAR_AXI_VDMA_9_INCLUDE_INTERNAL_GENLOCK,
		XPAR_AXI_VDMA_9_S2MM_SOF_ENABLE,
		XPAR_AXI_VDMA_9_M_AXIS_MM2S_TDATA_WIDTH,
		XPAR_AXI_VDMA_9_S_AXIS_S2MM_TDATA_WIDTH,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_1,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_5,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_6,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_7,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_9,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_13,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_14,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_INFO_15,
		XPAR_AXI_VDMA_9_ENABLE_DEBUG_ALL,
		XPAR_AXI_VDMA_9_ADDR_WIDTH
	}
};

#define INITIALIZATION_POLLING   100000

//--------------------------------------------------------------------------------------------------------------
XAxiVdma_Channel *XAxiVdma_GetChannel(XAxiVdma *InstancePtr,
        u16 Direction)
{

    if (Direction == XAXIVDMA_READ) {
        return &(InstancePtr->ReadChannel);
    }
    else if (Direction == XAXIVDMA_WRITE) {
        return &(InstancePtr->WriteChannel);
    }
    else {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "Invalid direction %x\r\n", Direction);

        return NULL;
    }
}
//--------------------------------------------------------------------------------------------------------------
static int XAxiVdma_Major(XAxiVdma *InstancePtr) {
    u32 Reg;

    Reg = XAxiVdma_ReadReg(InstancePtr->BaseAddr, XAXIVDMA_VERSION_OFFSET);

    return (int)((Reg & XAXIVDMA_VERSION_MAJOR_MASK) >>
              XAXIVDMA_VERSION_MAJOR_SHIFT);
}
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_CfgInitialize(XAxiVdma *InstancePtr, XAxiVdma_Config *CfgPtr,
				UINTPTR EffectiveAddr)
{
	XAxiVdma_Channel *RdChannel;
	XAxiVdma_Channel *WrChannel;
	int Polls;

	// // Validate parameters */
	// Xil_AssertNonvoid(InstancePtr != NULL);
	// Xil_AssertNonvoid(CfgPtr != NULL);

	// Initially, no interrupt callback functions
	InstancePtr->ReadCallBack.CompletionCallBack = 0x0;
	InstancePtr->ReadCallBack.ErrCallBack = 0x0;
	InstancePtr->WriteCallBack.CompletionCallBack = 0x0;
	InstancePtr->WriteCallBack.ErrCallBack = 0x0;

	InstancePtr->BaseAddr = EffectiveAddr;
	InstancePtr->MaxNumFrames = CfgPtr->MaxFrameStoreNum;
	InstancePtr->HasMm2S = CfgPtr->HasMm2S;
	InstancePtr->HasS2Mm = CfgPtr->HasS2Mm;
	InstancePtr->UseFsync = CfgPtr->UseFsync;
	InstancePtr->InternalGenLock = CfgPtr->InternalGenLock;
	InstancePtr->AddrWidth = CfgPtr->AddrWidth;

	if (XAxiVdma_Major(InstancePtr) < 3) {
		InstancePtr->HasSG = 1;
	}
	else {
		InstancePtr->HasSG = CfgPtr->HasSG;
	}

	// The channels are not valid until being initialized
	RdChannel = XAxiVdma_GetChannel(InstancePtr, XAXIVDMA_READ);
	RdChannel->IsValid = 0;

	WrChannel = XAxiVdma_GetChannel(InstancePtr, XAXIVDMA_WRITE);
	WrChannel->IsValid = 0;

	if (InstancePtr->HasMm2S) {
		RdChannel->direction = XAXIVDMA_READ;
		RdChannel->ChanBase = InstancePtr->BaseAddr + XAXIVDMA_TX_OFFSET;
		RdChannel->InstanceBase = InstancePtr->BaseAddr;
		RdChannel->HasSG = InstancePtr->HasSG;
		RdChannel->IsRead = 1;
		RdChannel->StartAddrBase = InstancePtr->BaseAddr +
		                              XAXIVDMA_MM2S_ADDR_OFFSET;

		RdChannel->NumFrames = CfgPtr->MaxFrameStoreNum;

		// Flush on Sync */
		RdChannel->FlushonFsync = CfgPtr->FlushonFsync;

		// Dynamic Line Buffers Depth */
		RdChannel->LineBufDepth = CfgPtr->Mm2SBufDepth;
		if(RdChannel->LineBufDepth > 0) {
			RdChannel->LineBufThreshold =
				XAxiVdma_ReadReg(RdChannel->ChanBase,
					XAXIVDMA_BUFTHRES_OFFSET);
			xdbg_printf(XDBG_DEBUG_GENERAL,
				"Read Channel Buffer Threshold %d bytes\n\r",
				RdChannel->LineBufThreshold);
		}
		RdChannel->HasDRE = CfgPtr->HasMm2SDRE;
		RdChannel->WordLength = CfgPtr->Mm2SWordLen >> 3;
		RdChannel->StreamWidth = CfgPtr->Mm2SStreamWidth >> 3;
		RdChannel->AddrWidth = InstancePtr->AddrWidth;

		// Internal GenLock */
		RdChannel->GenLock = CfgPtr->Mm2SGenLock;

		// Debug Info Parameter flags */
		if (!CfgPtr->EnableAllDbgFeatures) {
			if (CfgPtr->Mm2SThresRegEn) {
				RdChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_THRESHOLD_REG;
			}

			if (CfgPtr->Mm2SFrmStoreRegEn) {
				RdChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_FRMSTORE_REG;
			}

			if (CfgPtr->Mm2SDlyCntrEn) {
				RdChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_DLY_CNTR;
			}

			if (CfgPtr->Mm2SFrmCntrEn) {
				RdChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_FRM_CNTR;
			}

		} else {
			RdChannel->DbgFeatureFlags =
				XAXIVDMA_ENABLE_DBG_ALL_FEATURES;
		}

		XAxiVdma_ChannelInit(RdChannel);

		XAxiVdma_ChannelReset(RdChannel);

		// At time of initialization, no transfers are going on, reset is expected to be quick
		Polls = INITIALIZATION_POLLING;

		while (Polls && XAxiVdma_ChannelResetNotDone(RdChannel)) {
			Polls -= 1;
		}

		if (!Polls) {
			xdbg_printf(XDBG_DEBUG_ERROR,
			    "Read channel reset failed %x\n\r",
			    (unsigned int)XAxiVdma_ChannelGetStatus(RdChannel));

			return XST_FAILURE;
		}
	}

	if (InstancePtr->HasS2Mm) {
		WrChannel->direction = XAXIVDMA_WRITE;
		WrChannel->ChanBase = InstancePtr->BaseAddr + XAXIVDMA_RX_OFFSET;
		WrChannel->InstanceBase = InstancePtr->BaseAddr;
		WrChannel->HasSG = InstancePtr->HasSG;
		WrChannel->IsRead = 0;
		WrChannel->StartAddrBase = InstancePtr->BaseAddr +
		                                 XAXIVDMA_S2MM_ADDR_OFFSET;
		WrChannel->NumFrames = CfgPtr->MaxFrameStoreNum;
		WrChannel->AddrWidth = InstancePtr->AddrWidth;

		// Flush on Sync
		WrChannel->FlushonFsync = CfgPtr->FlushonFsync;

		// Dynamic Line Buffers Depth
		WrChannel->LineBufDepth = CfgPtr->S2MmBufDepth;
		if(WrChannel->LineBufDepth > 0) {
			WrChannel->LineBufThreshold =
				XAxiVdma_ReadReg(WrChannel->ChanBase,
					XAXIVDMA_BUFTHRES_OFFSET);
			xdbg_printf(XDBG_DEBUG_GENERAL,
				"Write Channel Buffer Threshold %d bytes\n\r",
				WrChannel->LineBufThreshold);
		}
		WrChannel->HasDRE = CfgPtr->HasS2MmDRE;
		WrChannel->WordLength = CfgPtr->S2MmWordLen >> 3;
		WrChannel->StreamWidth = CfgPtr->S2MmStreamWidth >> 3;

		// Internal GenLock
		WrChannel->GenLock = CfgPtr->S2MmGenLock;

		// Frame Sync Source Selection
		WrChannel->S2MmSOF = CfgPtr->S2MmSOF;

		// Debug Info Parameter flags
		if (!CfgPtr->EnableAllDbgFeatures) {
			if (CfgPtr->S2MmThresRegEn) {
				WrChannel->DbgFeatureFlags |=
					 XAXIVDMA_ENABLE_DBG_THRESHOLD_REG;
			}

			if (CfgPtr->S2MmFrmStoreRegEn) {
				WrChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_FRMSTORE_REG;
			}

			if (CfgPtr->S2MmDlyCntrEn) {
				WrChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_DLY_CNTR;
			}

			if (CfgPtr->S2MmFrmCntrEn) {
				WrChannel->DbgFeatureFlags |=
					XAXIVDMA_ENABLE_DBG_FRM_CNTR;
			}

		} else {
			WrChannel->DbgFeatureFlags =
					XAXIVDMA_ENABLE_DBG_ALL_FEATURES;
		}

		XAxiVdma_ChannelInit(WrChannel);

		XAxiVdma_ChannelReset(WrChannel);

		// At time of initialization, no transfers are going on, reset is expected to be quick
		Polls = INITIALIZATION_POLLING;

		while (Polls && XAxiVdma_ChannelResetNotDone(WrChannel)) {
			Polls -= 1;
		}

		if (!Polls) {
			xdbg_printf(XDBG_DEBUG_ERROR,
			    "Write channel reset failed %x\n\r",
			    (unsigned int)XAxiVdma_ChannelGetStatus(WrChannel));

			return XST_FAILURE;
		}
	}

	InstancePtr->IsReady = XAXIVDMA_DEVICE_READY;

	return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
XAxiVdma_Config *XAxiVdma_LookupConfig(u16 DeviceId)
{
    // extern XAxiVdma_Config XAxiVdma_ConfigTable[];
    XAxiVdma_Config *CfgPtr = NULL;
    u32 i;

    for (i = 0U; i < XPAR_XAXIVDMA_NUM_INSTANCES; i++) {
        if (XAxiVdma_ConfigTable[i].DeviceId == DeviceId) {
            CfgPtr = &XAxiVdma_ConfigTable[i];
            break;
        }
    }

    return CfgPtr;
}
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_DmaConfig(XAxiVdma *InstancePtr, u16 Direction,
        XAxiVdma_DmaSetup *DmaConfigPtr)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel) {
        return XST_INVALID_PARAM;
    }


    if (Channel->IsValid) {

        return XAxiVdma_ChannelConfig(Channel,
            (XAxiVdma_ChannelSetup *)DmaConfigPtr);
    }
    else {
        return XST_DEVICE_NOT_FOUND;
    }
}
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_DmaSetBufferAddr(XAxiVdma *InstancePtr, u16 Direction, UINTPTR *BufferAddrSet)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel) {
        return XST_INVALID_PARAM;
    }

    if (Channel->IsValid) {
        return XAxiVdma_ChannelSetBufferAddr(Channel, BufferAddrSet, Channel->NumFrames);
    }
    else {
        return XST_DEVICE_NOT_FOUND;
    }
}
//--------------------------------------------------------------------------------------------------------------
int XAxiVdma_DmaStart(XAxiVdma *InstancePtr, u16 Direction)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel) {
        return XST_INVALID_PARAM;
    }

    if (Channel->IsValid) {
        return XAxiVdma_ChannelStart(Channel);
    }
    else {
        return XST_DEVICE_NOT_FOUND;
    }
}
//--------------------------------------------------------------------------------------------------------------
void XAxiVdma_DmaStop(XAxiVdma *InstancePtr, u16 Direction)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel)
    {
        return;
    }

    if (Channel->IsValid)
    {
        XAxiVdma_ChannelStop(Channel);
    }

    return;
}
//--------------------------------------------------------------------------------------------------------------
void XAxiVdma_DmaRegisterDump(XAxiVdma *InstancePtr, u16 Direction)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel) {
        return;
    }

    if (Channel->IsValid) {
        XAxiVdma_ChannelRegisterDump(Channel);
    }

    return;
}
//--------------------------------------------------------------------------------------------------------------
void VDMA_Config_Dump(XAxiVdma_Config *ptr)
{
	xil_printf("---------------------------------------------\r\n");
	xil_printf("Base Address = %x\r\n", ptr->BaseAddress);
	xil_printf("Max Frame Store Num = %d\r\n", ptr->HasMm2S);
	xil_printf("HasMm2S = %d\r\n", ptr->HasMm2S);
	xil_printf("HasMm2SDRE = %d\r\n", ptr->HasMm2SDRE);
	xil_printf("Mm2SWordLen = %d\r\n", ptr->Mm2SWordLen);
	xil_printf("HasS2Mm = %d\r\n", ptr->HasS2Mm);
	xil_printf("HasS2MmDRE = %d\r\n", ptr->HasS2MmDRE);
	xil_printf("S2MmWordLen = %d\r\n", ptr->S2MmWordLen);
	xil_printf("HasSG = %d\r\n", ptr->HasSG);
	xil_printf("EnableVIDParamRead = %d\r\n", ptr->EnableVIDParamRead);
	xil_printf("UseFsync = %d\r\n", ptr->UseFsync);
	xil_printf("FlushonFsync = %d\r\n", ptr->FlushonFsync);
	xil_printf("Mm2SBufDepth = %d\r\n", ptr->Mm2SBufDepth);
	xil_printf("S2MmBufDepth = %d\r\n", ptr->S2MmBufDepth);
	xil_printf("Mm2SGenLock = %d\r\n", ptr->Mm2SGenLock);
	xil_printf("S2MmGenLock = %d\r\n", ptr->S2MmGenLock);
	xil_printf("InternalGenLock = %d\r\n", ptr->InternalGenLock);
	xil_printf("S2MmSOF = %d\r\n", ptr->S2MmSOF);
	xil_printf("Mm2SStreamWidth = %d\r\n", ptr->Mm2SStreamWidth);
	xil_printf("S2MmStreamWidth = %d\r\n", ptr->S2MmStreamWidth);
	xil_printf("Mm2SThresRegEn = %d\r\n", ptr->Mm2SThresRegEn);
	xil_printf("Mm2SFrmStoreRegEn = %d\r\n", ptr->Mm2SFrmStoreRegEn);
	xil_printf("Mm2SDlyCntrEn = %d\r\n", ptr->Mm2SDlyCntrEn);
	xil_printf("Mm2SFrmCntrEn = %d\r\n", ptr->Mm2SFrmCntrEn);
	xil_printf("S2MmThresRegEn = %d\r\n", ptr->S2MmThresRegEn);
	xil_printf("S2MmFrmStoreRegEn = %d\r\n", ptr->S2MmFrmStoreRegEn);
	xil_printf("S2MmDlyCntrEn = %d\r\n", ptr->S2MmDlyCntrEn);
	xil_printf("S2MmFrmCntrEn = %d\r\n", ptr->S2MmFrmCntrEn);
	xil_printf("EnableAllDbgFeatures = %d\r\n", ptr->EnableAllDbgFeatures);
	xil_printf("AddrWidth = %d\r\n", ptr->AddrWidth);
	xil_printf("---------------------------------------------\r\n");
}
//--------------------------------------------------------------------------------------------------------------
void VDMA_Setup_Dump(XAxiVdma_DmaSetup *ptr)
{
	int i;

	xil_printf("---------------------------------------------\r\n");
	xil_printf("VertSizeInput = %d\r\n", ptr->VertSizeInput);
	xil_printf("HoriSizeInput = %d\r\n", ptr->HoriSizeInput);
	xil_printf("Stride = %d\r\n", ptr->Stride);
	xil_printf("FrameDelay = %d\r\n", ptr->FrameDelay);
	xil_printf("EnableCircularBuf = %d\r\n", ptr->EnableCircularBuf);
	xil_printf("EnableSync = %d\r\n", ptr->EnableSync);
	xil_printf("PointNum = %d\r\n", ptr->PointNum);
	xil_printf("EnableFrameCounter = %d\r\n", ptr->EnableFrameCounter);
	xil_printf("FixedFrameStoreAddr = %d\r\n", ptr->FixedFrameStoreAddr);
	xil_printf("GenLockRepeat = %d\r\n", ptr->GenLockRepeat);

	for(i = 0; i < XAXIVDMA_MAX_FRAMESTORE; i ++)
	{
		xil_printf("%d FrameStoreStartAddr = %x\r\n", i, ptr->FrameStoreStartAddr[i]);
	}
	xil_printf("---------------------------------------------\r\n");
}
//--------------------------------------------------------------------------------------------------------------
void VDMA_Check_Errors(u32 addr)
{

}
//--------------------------------------------------------------------------------------------------------------
void VDMA_Control_Dump(u32 addr)
{
	u32 *ptr;
	u32 i;
//

	xil_printf("---------------------------------------------\r\n");
	for(i = 0; i < 0xFF; i += 4)
	{
		xil_printf("%x Address = %x\r\n", addr+i, XAxiVdma_ReadReg(addr, i));
		ptr++;
	}
	xil_printf("---------------------------------------------\r\n");
}

//--------------------------------------------------------------------------------------------------------------
/*
int activate_vdma_1(int base, int hsize, int vsize, uint32_t *vdma1_base)
{
    int status;
    XAxiVdma_Config *Config;
	u32 Addr, storage_offset;

	xil_printf("VDMA1 Init Start\n");

    Config = XAxiVdma_LookupConfig(1);
    if (!Config) {
        xil_printf("No video DMA1 found\r\n");
        return XST_FAILURE;
    }else
	{
		xil_printf("Video DMA1 config address = %x\n", Config->BaseAddress);
	}

    status = XAxiVdma_CfgInitialize(&InstancePtr1, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }else
	{
		xil_printf("Configuration \n");
	}

    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);
//  printf("hsize: %d  Width: %d\n", hsize, Config->Mm2SStreamWidth);

    ReadCfg1.VertSizeInput       = vsize;
    ReadCfg1.HoriSizeInput       = stride;
    ReadCfg1.Stride              = stride;
    ReadCfg1.FrameDelay          = 0;      // This example does not test frame delay
    ReadCfg1.EnableCircularBuf   = 1;      // Only 1 buffer, continuous loop
    ReadCfg1.EnableSync          = 0;      // Gen-Lock
    ReadCfg1.PointNum            = 0;
    ReadCfg1.EnableFrameCounter  = 0;      // Endless transfers
    ReadCfg1.FixedFrameStoreAddr = 0;      // We are not doing parking

    status = XAxiVdma_DmaConfig(&InstancePtr1, XAXIVDMA_READ, &ReadCfg1);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

	// storage_offset = ReadCfg1.Stride;
	// Addr = VDMA1_BASE_MEM  + storage_offset;
	// ReadCfg1.FrameStoreStartAddr[0] = Addr;


    // Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer
    status = XAxiVdma_DmaSetBufferAddr(&InstancePtr1, XAXIVDMA_READ, (UINTPTR *)vdma1_base);
	// status = XAxiVdma_DmaSetBufferAddr(&InstancePtr1, XAXIVDMA_READ, ReadCfg1.FrameStoreStartAddr);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }


    // Start the Read channel of DMA Engine
    DBG("Start VDMA1 \n");
    status = XAxiVdma_DmaStart(&InstancePtr1, XAXIVDMA_READ);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    // ************ DMA engine start done ***************
	VDMA_Config_Dump(Config);
	VDMA_Setup_Dump(&ReadCfg1);
	xil_printf("VDMA1 Engine Start done\n");
    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int activate_vdma_3(int base, int hsize, int vsize, uint32_t *fb_mem)
{
    int status;
    XAxiVdma_Config *Config;

	xil_printf("VDMA3 Init Start .....\n");
    Config = XAxiVdma_LookupConfig(3);
    if (!Config) {
        xil_printf("No video DMA3 found\r\n");
        return XST_FAILURE;
    }else
	{
		xil_printf("Video DMA3 fount\n");
	}

    status = XAxiVdma_CfgInitialize(&InstancePtr3, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }
	u32 stride = hsize * (Config->Mm2SStreamWidth>>3);

    ReadCfg3.VertSizeInput       = vsize;
    ReadCfg3.HoriSizeInput       = stride;
    ReadCfg3.Stride              = stride;
    ReadCfg3.FrameDelay          = 0;      // This example does not test frame delay
    ReadCfg3.EnableCircularBuf   = 1;      // Only 1 buffer, continuous loop
    ReadCfg3.EnableSync          = 0;      // Gen-Lock
    ReadCfg3.PointNum            = 0;
    ReadCfg3.EnableFrameCounter  = 0;      // Endless transfers
    ReadCfg3.FixedFrameStoreAddr = 0;      // We are not doing parking

    status = XAxiVdma_DmaConfig(&InstancePtr3, XAXIVDMA_READ, &ReadCfg3);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

    // Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer
    status = XAxiVdma_DmaSetBufferAddr(&InstancePtr3, XAXIVDMA_READ, (UINTPTR *)fb_mem);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }

    // Start the Read channel of DMA Engine
    DBG("Start VDMA3\n");
    status = XAxiVdma_DmaStart(&InstancePtr3, XAXIVDMA_READ);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    // ************ DMA engine start done ***************
	xil_printf("VDMA3 Engine Start done\n");
	VDMA_Config_Dump(Config);
	VDMA_Setup_Dump(&ReadCfg3);
    return XST_SUCCESS;
}

*/
int activate_vdma_4(int base, int hsize, int vsize, uint32_t *vdma4_base, int Mode)
{
    int status;
    XAxiVdma_Config *Config;

	xil_printf("VDMA4 Init Start\n");
    Config = XAxiVdma_LookupConfig(1);
    if (!Config) {
        xil_printf("No video DMA4 found\r\n");
        return XST_FAILURE;
    }else
	{
		xil_printf("VDMA4 Config ok %x\n", Config->BaseAddress);
	}

    status = XAxiVdma_CfgInitialize(&InstancePtr4, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }else
	{
		xil_printf("Configuration Initialization ok\n");
	}
    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);

    ReadCfg4.VertSizeInput       = vsize;
    ReadCfg4.HoriSizeInput       = stride;
    ReadCfg4.Stride              = stride;
    ReadCfg4.FrameDelay          = 0;      // This example does not test frame delay
    ReadCfg4.EnableCircularBuf   = 1;      // Only 1 buffer, continuous loop
    ReadCfg4.EnableSync          = 0;      // Gen-Lock
    ReadCfg4.PointNum            = 0;
    ReadCfg4.EnableFrameCounter  = 0;      // Endless transfers
    ReadCfg4.FixedFrameStoreAddr = 0;      // We are not doing parking

    status = XAxiVdma_DmaConfig(&InstancePtr4, XAXIVDMA_WRITE, &ReadCfg4);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }else
	{
		xil_printf("Read channel config\n");
	}

    // Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer
    status = XAxiVdma_DmaSetBufferAddr(&InstancePtr4, XAXIVDMA_WRITE, (UINTPTR *)vdma4_base);
	// status = XAxiVdma_DmaSetBufferAddr(&InstancePtr4, XAXIVDMA_WRITE, ReadCfg4.FrameStoreStartAddr);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }

    // Start the Read channel of DMA Engine
    DBG("Start VDMA4\n");
    status = XAxiVdma_DmaStart(&InstancePtr4, XAXIVDMA_WRITE);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    // ************ DMA engine start done ***************

	VDMA_Config_Dump(Config);
	VDMA_Setup_Dump(&ReadCfg4);
	xil_printf("VDMA4 Engine Start done  ... \n");
    return XST_SUCCESS;
}

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
int Vdma_WrSetup(XAxiVdma *wInsPtr, XAxiVdma_DmaSetup *wSetupCfg, u16 Id, int hSize, int vSize, int hOffset, int vOffset, u32 wfb_mem)
{
	int status;
	u32 stride;
	XAxiVdma_Config *wConfig;
	u32 Addr;
	u32 storage_offset;

	xil_printf("[VDMA] : DMA%d Init-----------------------\r\n", Id);

	wConfig = XAxiVdma_LookupConfig(Id);
	if (!wConfig)
	{
		xil_printf("[VDMA] : Video DMA%d not found\r\n", Id);
		return XST_FAILURE;
	}else
		xil_printf("[VDMA] : Video DMA%d found\r\n", Id);

	status = XAxiVdma_CfgInitialize(wInsPtr, wConfig, wConfig->BaseAddress);
	if (status != XST_SUCCESS) {
		xil_printf("[VDMA] : Configuration Initialization failed, status: 0x%X\r\n", status);
		return status;
	}else
	{
		xil_printf("[VDMA] : Configuration initialization = %x\r\n", wConfig->BaseAddress);
	}

	wSetupCfg->VertSizeInput       = vSize;
	if(Id == VDMA6_FRAME_DEVICE_ID)
		wSetupCfg->HoriSizeInput   = hSize * 2;
	else
		wSetupCfg->HoriSizeInput   = hSize * 2;

	if(Id == VDMA6_FRAME_DEVICE_ID)
		wSetupCfg->Stride              = 1920 * 2;
	else
		wSetupCfg->Stride              = 1920 * 2;
	// wSetupCfg->Stride              = stride;

	wSetupCfg->FrameDelay          = 0;      /* This example does not test frame delay */
	wSetupCfg->EnableCircularBuf   = 1;      /* Only 1 buffer, continuous loop */
	wSetupCfg->EnableSync          = 0;      /* Gen-Lock */
	wSetupCfg->PointNum            = 0;
	wSetupCfg->EnableFrameCounter  = 0;      /* Endless transfers */
	wSetupCfg->FixedFrameStoreAddr = 0;      /* We are not doing parking */

	status = XAxiVdma_DmaConfig(wInsPtr, XAXIVDMA_WRITE, wSetupCfg);
	if (status != XST_SUCCESS) {
		xil_printf("[VDMA] : Write channel config failed, status: 0x%X\r\n", status);
		return status;
	}else
	{
		xil_printf("[VDMA] : Write Channel Config OK\r\n");
	}
	/*
	storage_offset = ReadCfg4.Stride;
	if(Mode == 0)			// FULL HD
		Addr = wfb_mem; // + storage_offset + ((1920 * 180 * 4) + (320 *4));
	else 					// HD
		Addr = wfb_mem; //  + storage_offset + ((1920 * gDb.yImageStart * 2) + (gDb.xImageStart *2));
*/

	if(Id == VDMA5_FRAME_DEVICE_ID)
	{
		Addr = wfb_mem +  ((2048 * vOffset) + (hOffset *2));
		wSetupCfg->FrameStoreStartAddr[0] = Addr;
	}else
	{
		Addr = wfb_mem;
		wSetupCfg->FrameStoreStartAddr[0] = Addr;
	}

	status = XAxiVdma_DmaSetBufferAddr(wInsPtr, XAXIVDMA_WRITE, wSetupCfg->FrameStoreStartAddr);
	if (status != XST_SUCCESS) {
		xil_printf("[VDMA] : Write channel set buffer address failed, status: 0x%X\r\n", status);
		return status;
	}else
		xil_printf("[VDMA] : Write Channel Set Buffer Address\r\n");

	status = XAxiVdma_DmaStart(wInsPtr, XAXIVDMA_WRITE);
	if (status != XST_SUCCESS) {
		xil_printf("[VDMA] : Failed to start DMA engine (Write channel), status: 0x%X\r\n", status);
		return status;
	}else
		xil_printf("[VDMA] : Start DMA Engine(Write channel)\r\n");

	// VDMA_Config_Dump(wConfig);
	// VDMA_Setup_Dump(wSetupCfg);
	VDMA_Control_Dump(wConfig->BaseAddress);

	return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int Vdma_wSetup(XAxiVdma *wInsPtr, XAxiVdma_DmaSetup *wSetupCfg, int Id, int hsize, int vsize, int hOffset, int vOffset, uint32_t *wvdma_base)
{
 	int status;
    XAxiVdma_Config *wConfig;
	u32 Addr;

	xil_printf("\r\n-----------------------------------------------------------------\r\n");
	xil_printf("[VDMA-W OK] : VDMA%d Init Start\n", Id);
    wConfig = XAxiVdma_LookupConfig(Id);
    if (!wConfig) {
        xil_printf("[VDMA-W NO] : No video DMA%d found\r\n", Id);
        return XST_FAILURE;
    }else
	{
		xil_printf("[VDMA-W OK] : VDMA%d Config ok\n", Id);
	}
	vdma_orgBase = wConfig->BaseAddress;
	Vdma_Open(wConfig->BaseAddress);

    status = XAxiVdma_CfgInitialize(wInsPtr, wConfig, wConfig->BaseAddress);
    if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA-W NO] : Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }else
	{
		xil_printf("[VDMA-W OK] : Configuration Initialization ok\n");
	}

    wSetupCfg->VertSizeInput       = vsize;
    wSetupCfg->HoriSizeInput       = hsize * 2;
    wSetupCfg->Stride              = 1920 * 2;
    wSetupCfg->FrameDelay          = 0;      // This example does not test frame delay
    wSetupCfg->EnableCircularBuf   = 1;      // Only 1 buffer, continuous loop
    wSetupCfg->EnableSync          = 0;      // Gen-Lock
    wSetupCfg->PointNum            = 0;
    wSetupCfg->EnableFrameCounter  = 0;      // Endless transfers
    wSetupCfg->FixedFrameStoreAddr = 0;      // We are not doing parking

    status = XAxiVdma_DmaConfig(wInsPtr, XAXIVDMA_WRITE, wSetupCfg);
    if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA-W NO] : Read channel config failed, status: 0x%X\r\n", status);
        // return status;
    }else
	{
		xil_printf("[VDMA-W OK] : Read channel config\n");
	}

	if(Id == VDMA5_FRAME_DEVICE_ID)
	{
		Addr = wfb5_mem + ((1920 * vOffset * 2) + (hOffset * 2));
		wSetupCfg->FrameStoreStartAddr[0] = Addr;

		status = XAxiVdma_DmaSetBufferAddr(wInsPtr, XAXIVDMA_WRITE, wSetupCfg->FrameStoreStartAddr);
	}else
	{
    	status = XAxiVdma_DmaSetBufferAddr(wInsPtr, XAXIVDMA_WRITE, (UINTPTR *)wvdma_base);
	}

    if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA-W NO] : Read channel set buffer address failed, status: 0x%X\r\n", status);
        // return status;
    }

    status = XAxiVdma_DmaStart(wInsPtr, XAXIVDMA_WRITE);
    if (status != XST_SUCCESS) {
        xil_printf("[VDMA-W NO] : Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        // return status;
    }

	// VDMA_Config_Dump(wConfig);
	// VDMA_Setup_Dump(wSetupCfg);
	// VDMA_Control_Dump(wConfig->BaseAddress);
	Vdma_Close();
	xil_printf("=================================================================\r\n");
    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int Vdma_rSetup(XAxiVdma *InsPtr, XAxiVdma_DmaSetup *SetupCfg, int Id, int hsize, int vsize, uint32_t *rvdma_base)
{
    int status;
    XAxiVdma_Config *rConfig;

	xil_printf("\r\n-----------------------------------------------------------------\r\n");
	xil_printf("[VDMA OK] : VDMA%d Init Start\r\n", Id);

    rConfig = XAxiVdma_LookupConfig(Id);
    if (!rConfig)
	{
        xil_printf("[VDMA NO] : No video DMA%d found\r\n", Id);
        return XST_FAILURE;
    }else
	{
		xil_printf("[VDMA OK] : Video DMA%d config address = %x\n", Id, rConfig->BaseAddress);
	}

	vdma_orgBase = rConfig->BaseAddress;
	Vdma_Open(rConfig->BaseAddress);

    status = XAxiVdma_CfgInitialize(InsPtr, rConfig, rConfig->BaseAddress);
    if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA NO] : Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }else
	{
		xil_printf("[VDMA OK] : Configuration Initialization OK\n");
	}

	if(Id == VDMA3_FRAME_DEVICE_ID)
	{
    	SetupCfg->VertSizeInput       = vsize;
    	SetupCfg->HoriSizeInput       = hsize * 4;
    	SetupCfg->Stride              = 1920 * 4;
	}else
	{
		SetupCfg->VertSizeInput       = vsize;
    	SetupCfg->HoriSizeInput       = hsize * 2;
    	SetupCfg->Stride              = 1920 * 2;
	}
    SetupCfg->FrameDelay          = 0;      // This example does not test frame delay */
    SetupCfg->EnableCircularBuf   = 1;      // Only 1 buffer, continuous loop */
    SetupCfg->EnableSync          = 1;      // Gen-Lock */
    SetupCfg->PointNum            = 0;
    SetupCfg->EnableFrameCounter  = 0;      // Endless transfers */
    SetupCfg->FixedFrameStoreAddr = 0;      // We are not doing parking */


    status = XAxiVdma_DmaConfig(InsPtr, XAXIVDMA_READ, SetupCfg);
    if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA NO] : Read channel config failed, status: 0x%X\r\n", status);
        // return status;
    }

    status = XAxiVdma_DmaSetBufferAddr(InsPtr, XAXIVDMA_READ, (UINTPTR *)rvdma_base);
    if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA NO] : Read channel set buffer address failed, status: 0x%X\r\n", status);
		// return status;
    }
    status = XAxiVdma_DmaStart(InsPtr, XAXIVDMA_READ);

	if (status != XST_SUCCESS)
	{
        xil_printf("[VDMA NO] : Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        // return status;
    }

	// VDMA_Config_Dump(rConfig);
	// VDMA_Setup_Dump(SetupCfg);
	// VDMA_Control_Dump(rConfig->BaseAddress);
	xil_printf("[VDMA OK] : VDMA%d Engine Start done\r\n", Id);
	xil_printf("=================================================================\r\n");
	Vdma_Close();
    return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int Vdma_Stop(int Id)
{
	switch(Id)
	{
		case VDMA1_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&rInstancePtr1, XAXIVDMA_READ);
			break;

		case VDMA3_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&rInstancePtr3, XAXIVDMA_READ);
			break;

		case VDMA9_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&rInstancePtr9, XAXIVDMA_READ);
			break;

		case VDMA7_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&rInstancePtr7, XAXIVDMA_READ);
			break;

		case VDMA8_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&rInstancePtr8, XAXIVDMA_READ);
			break;

		case VDMA4_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&wInstancePtr4, XAXIVDMA_WRITE);
			break;

		case VDMA5_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&wInstancePtr5, XAXIVDMA_WRITE);
			break;

		case VDMA6_FRAME_DEVICE_ID:
			XAxiVdma_DmaStop(&wInstancePtr6, XAXIVDMA_WRITE);
			break;

		case 99:
			break;

		default:
			break;
	}

	return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------
int Vdma_Start(int Id)
{
	switch(Id)
	{
		case VDMA1_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&rInstancePtr1, XAXIVDMA_READ);
			break;

		case VDMA3_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&rInstancePtr3, XAXIVDMA_READ);
			break;

		case VDMA9_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&rInstancePtr9, XAXIVDMA_READ);
			break;

		case VDMA7_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&rInstancePtr7, XAXIVDMA_READ);
			break;

		case VDMA8_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&rInstancePtr8, XAXIVDMA_READ);
			break;

		case VDMA4_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&wInstancePtr4, XAXIVDMA_WRITE);
			break;

		case VDMA5_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&wInstancePtr5, XAXIVDMA_WRITE);
			break;

		case VDMA6_FRAME_DEVICE_ID:
			XAxiVdma_DmaStart(&wInstancePtr6, XAXIVDMA_WRITE);
			break;

		case 99:
			break;

		default:
			break;
	}

	return XST_SUCCESS;
}
//--------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------
int Vdma_SetActive(int Id, int hsize, int vsize, int hoffset, int voffset, uint32_t *vdma_base)
{
	int status;

	rfb1_mem = RVDMA1_BASE_MEM;
	wfb4_mem = WVDMA4_BASE_MEM;
	wfb5_mem = WVDMA5_BASE_MEM;
	wfb6_mem = WVDMA6_BASE_MEM;
	rfb7_mem = RVDMA7_BASE_MEM;
	rfb8_mem = RVDMA8_BASE_MEM;
	rfb9_mem = RVDMA9_BASE_MEM;

	switch(Id)
	{
		case VDMA1_FRAME_DEVICE_ID:
			status = Vdma_rSetup(&rInstancePtr1, &rReadCfg1, VDMA1_FRAME_DEVICE_ID, hsize, vsize, &rfb1_mem);
			break;

		case VDMA3_FRAME_DEVICE_ID:
			status = Vdma_rSetup(&rInstancePtr3, &rReadCfg3, VDMA3_FRAME_DEVICE_ID, hsize, vsize, vdma_base);
			break;

		case VDMA9_FRAME_DEVICE_ID:
			status = Vdma_rSetup(&rInstancePtr9, &rReadCfg3, VDMA9_FRAME_DEVICE_ID, hsize, vsize, &rfb9_mem);
			break;

		case VDMA7_FRAME_DEVICE_ID:
			status = Vdma_rSetup(&rInstancePtr7, &rReadCfg7, VDMA7_FRAME_DEVICE_ID, hsize, vsize, &rfb7_mem);
			break;

		case VDMA8_FRAME_DEVICE_ID:
			status = Vdma_rSetup(&rInstancePtr8, &rReadCfg8, VDMA8_FRAME_DEVICE_ID, hsize, vsize, &rfb8_mem);
			break;

		case VDMA4_FRAME_DEVICE_ID:
			status = Vdma_wSetup(&wInstancePtr4, &wReadCfg4, VDMA4_FRAME_DEVICE_ID, hsize, vsize, 0, 0, &wfb4_mem);
			break;

		case VDMA5_FRAME_DEVICE_ID:
			status = Vdma_wSetup(&wInstancePtr5, &wReadCfg5, VDMA5_FRAME_DEVICE_ID, hsize, vsize, hoffset, voffset, &wfb5_mem);
			break;

		case VDMA6_FRAME_DEVICE_ID:
			status = Vdma_wSetup(&wInstancePtr6, &wReadCfg6, VDMA6_FRAME_DEVICE_ID, hsize, vsize, 0, 0, &wfb6_mem);
			break;

		default:
			status = 0;
			break;
	}
	return status;
}
//--------------------------------------------------------------------------------------------------------------
int Vtc_Change(uint32_t vtcConfig, int Mode)
{
	Vtc_Open(vtcConfig);
	if(Mode == 1)		// FULL HD
	{
		XVtc_WriteReg(vtcConfig, (0x60), 0x43803C0);
		//XVtc_WriteReg(vtcConfig, (0x64), 0x04);
		XVtc_WriteReg(vtcConfig, (0x68), 0x02);
		XVtc_WriteReg(vtcConfig, (0x6C), 0x7F);
		XVtc_WriteReg(vtcConfig, (0x70), 0x44C);
		XVtc_WriteReg(vtcConfig, (0x74), 0x4650465);
		XVtc_WriteReg(vtcConfig, (0x78), 0x40203EC);
		XVtc_WriteReg(vtcConfig, (0x7C), 0x3C003C0);
		XVtc_WriteReg(vtcConfig, (0x80), 0x440043B);
		XVtc_WriteReg(vtcConfig, (0x84), 0x3C003C0);
		XVtc_WriteReg(vtcConfig, (0x88), 0x7800780);
		XVtc_WriteReg(vtcConfig, (0x8C), 0x440043B);
		XVtc_WriteReg(vtcConfig, (0x90), 0x7800780);

	}else				// 5:4
	{
		XVtc_WriteReg(vtcConfig, (0x60), 0x4000280);
		// XVtc_WriteReg(vtcConfig, (0x64), 0x00);
		XVtc_WriteReg(vtcConfig, (0x68), 0x02);
		XVtc_WriteReg(vtcConfig, (0x6C), 0x7F);
		XVtc_WriteReg(vtcConfig, (0x70), 0x48A);
		XVtc_WriteReg(vtcConfig, (0x74), 0x4670429);
		XVtc_WriteReg(vtcConfig, (0x78), 0x40203Ec);
		XVtc_WriteReg(vtcConfig, (0x7C), 0x2800280);
		XVtc_WriteReg(vtcConfig, (0x80), 0x41E041A);
		XVtc_WriteReg(vtcConfig, (0x84), 0x2800280);
		XVtc_WriteReg(vtcConfig, (0x88), 0x7800780);
		XVtc_WriteReg(vtcConfig, (0x8C), 0x440043B);
		XVtc_WriteReg(vtcConfig, (0x90), 0x7800780);
	}
	Vtc_Close();

	return 1;
}
//--------------------------------------------------------------------------------------------------------------
int Vtc_Enable(int Id, int Mode)
{
	switch(Id)
	{
		case XVTC1_DEVICE_ID:
			Vtc_Open(0x83C10000);
			if(Mode == 1)
				XVtc_WriteReg(0, (0x00), 0x7F7EF07);
			else
				XVtc_WriteReg(0, (0x00), 0x0);
			Vtc_Close();
			break;

		case XVTC2_DEVICE_ID:
			Vtc_Open(0x83C20000);
			if(Mode == 1)
				XVtc_WriteReg(0, (0x00), 0x7F7EF07);
			else
				XVtc_WriteReg(0, (0x00), 0x0);
			Vtc_Close();
			break;

		case XVTC5_DEVICE_ID:
			Vtc_Open(0x83C40000);
			if(Mode == 1)
				XVtc_WriteReg(0, (0x00), 0x7F7EF07);
			else
				XVtc_WriteReg(0, (0x00), 0x0);
			Vtc_Close();
			break;

		case XVTC4_DEVICE_ID:
			Vtc_Open(0x83C30000);
			if(Mode == 1)
				XVtc_WriteReg(0, (0x00), 0x7F7EF07);
			else
				XVtc_WriteReg(0, (0x00), 0x0);
			Vtc_Close();
			break;

		default:

			break;
	}
	return 1;
}
//--------------------------------------------------------------------------------------------------------------
int Vtc_Init(int Id, int Mode)
{
	vtc1_mem = 0x83C10000;
	vtc2_mem = 0x83C20000;
	vtc5_mem = 0x83C40000;
	vtc4_mem = 0x83C30000;

	switch(Id)
	{
		case XVTC1_DEVICE_ID:
			Vtc_Change(vtc1_mem, Mode);
			break;

		case XVTC2_DEVICE_ID:
			break;

		case XVTC5_DEVICE_ID:
			Vtc_Change(vtc5_mem, Mode);
			break;

		case XVTC4_DEVICE_ID:

			break;

		default:

			break;
	}
	return 1;
}
//--------------------------------------------------------------------------------------------------------------
int Vdma_Init(uint32_t *menu_addr)
{
	Vdma_SetActive(VDMA1_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
	Vdma_SetActive(VDMA3_FRAME_DEVICE_ID, 1920, 1080, 0, 0, menu_addr);
	Vdma_SetActive(VDMA9_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
	Vdma_SetActive(VDMA7_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
	Vdma_SetActive(VDMA8_FRAME_DEVICE_ID, 720,   480, 0, 0, 0);

	Vdma_SetActive(VDMA4_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
	Vdma_SetActive(VDMA5_FRAME_DEVICE_ID, 1536,  864, 0, 0, 0);
	Vdma_SetActive(VDMA6_FRAME_DEVICE_ID, 720,   480, 0, 0, 0);
	return 1;
}
