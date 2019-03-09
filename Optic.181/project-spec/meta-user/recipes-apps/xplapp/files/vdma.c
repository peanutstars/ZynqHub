#include <stdio.h>
#include <string.h>
#include "xilinx/xiltypes.h"
#include "xplioctl.h"
#include "debug.h"

#define VDMA0_BASE_MEM				0x60000000
#define VDMA1_BASE_MEM				0x70000000		// PL : Video In / Out
#define VDMA2_BASE_MEM				0x70000000
#define VDMA3_BASE_MEM				0x60000000		// PS : REAR Out
#define VDMA4_BASE_MEM				0x70000000		// PS : REAR Out

static XAxiVdma InstancePtr0;
static XAxiVdma InstancePtr1;
static XAxiVdma InstancePtr2;
static XAxiVdma InstancePtr3;
static XAxiVdma InstancePtr4;

static XAxiVdma_DmaSetup ReadCfg0;
static XAxiVdma_DmaSetup ReadCfg1;
static XAxiVdma_DmaSetup ReadCfg2;
static XAxiVdma_DmaSetup ReadCfg3;
static XAxiVdma_DmaSetup ReadCfg4;

static void XAxiVdma_BdSetNextPtr(XAxiVdma_Bd *BdPtr, u32 NextPtr);
static void XAxiVdma_BdWrite(XAxiVdma_Bd *BdPtr, int Offset, u32 Value);
static int XAxiVdma_BdSetVsize(XAxiVdma_Bd *BdPtr, int Vsize);
static int XAxiVdma_BdSetHsize(XAxiVdma_Bd *BdPtr, int Hsize);
static int XAxiVdma_BdSetStride(XAxiVdma_Bd *BdPtr, int Stride);
static int XAxiVdma_BdSetFrmDly(XAxiVdma_Bd *BdPtr, int FrmDly);
static void XAxiVdma_BdSetAddr(XAxiVdma_Bd *BdPtr, u32 Addr);

uint32_t XAxiVdma_ReadReg(UINTPTR base, int offset)
{
    uint32_t regv = 0;
    uint32_t _offset = (uint32_t)base + offset;
    if (reg_read32(_offset, &regv) != IOK) {
        ERR("READ32(offset:%08X)", _offset);
    }
    DBG("RD32:%08X -> %08X\n", _offset, regv);
    return regv;
}

void XAxiVdma_WriteReg(UINTPTR base, int offset, uint32_t data)
{
    uint32_t _offset = (uint32_t)base + offset;
    DBG("WR32:%08X <- %08X\n", _offset, data);
    if (reg_write32(_offset, data) != IOK) {
        ERR("WRITE32(offset:%08X, %08X)", _offset, data);
    }
}

/*****************************************************************************/
/*
 * Translate virtual address to physical address
 *
 * When port this driver to other RTOS, please change this definition to
 * be consistent with your target system.
 *
 * @param VirtAddr is the virtual address to work on
 *
 * @return
 *   The physical address of the virtual address
 *
 * @note
 *   The virtual address and the physical address are the same here.
 *
 *****************************************************************************/
#define XAXIVDMA_VIRT_TO_PHYS(VirtAddr) \
    (VirtAddr)

/*****************************************************************************/
/**
 * Set the channel to enable access to higher Frame Buffer Addresses (SG=0)
 *
 * @param Channel is the pointer to the channel to work on
 *
 *
 *****************************************************************************/
#define XAxiVdma_ChannelHiFrmAddrEnable(Channel) \
{ \
    XAxiVdma_WriteReg(Channel->ChanBase, \
            XAXIVDMA_HI_FRMBUF_OFFSET, XAXIVDMA_REGINDEX_MASK); \
}

/*****************************************************************************/
/**
 * Set the channel to disable access higher Frame Buffer Addresses (SG=0)
 *
 * @param Channel is the pointer to the channel to work on
 *
 *
 *****************************************************************************/
#define XAxiVdma_ChannelHiFrmAddrDisable(Channel) \
{ \
    XAxiVdma_WriteReg(Channel->ChanBase, \
        XAXIVDMA_HI_FRMBUF_OFFSET, (XAXIVDMA_REGINDEX_MASK >> 1)); \
}

/*****************************************************************************/
/**
 * Initialize a channel of a DMA engine
 *
 * This function initializes the BD ring for this channel
 *
 * @param Channel is the pointer to the DMA channel to work on
 *
 * @return
 *   None
 *
 *****************************************************************************/
 void XAxiVdma_ChannelInit(XAxiVdma_Channel *Channel)
 {
     int i;
     int NumFrames;
     XAxiVdma_Bd *FirstBdPtr = &(Channel->BDs[0]);
     XAxiVdma_Bd *LastBdPtr;

     /* Initialize the BD variables, so proper memory management
      * can be done
      */
     NumFrames = Channel->NumFrames;
     Channel->IsValid = 0;
     Channel->HeadBdPhysAddr = 0;
     Channel->HeadBdAddr = 0;
     Channel->TailBdPhysAddr = 0;
     Channel->TailBdAddr = 0;

     LastBdPtr = &(Channel->BDs[NumFrames - 1]);

     /* Setup the BD ring
      */
     memset((void *)FirstBdPtr, 0, NumFrames * sizeof(XAxiVdma_Bd));

     for (i = 0; i < NumFrames; i++) {
         XAxiVdma_Bd *BdPtr;
         XAxiVdma_Bd *NextBdPtr;

         BdPtr = &(Channel->BDs[i]);

         /* The last BD connects to the first BD
          */
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

     /* Setup the BD addresses so that access the head/tail BDs fast
      *
      */
     Channel->HeadBdAddr = (UINTPTR)FirstBdPtr;
     Channel->HeadBdPhysAddr = XAXIVDMA_VIRT_TO_PHYS((UINTPTR)FirstBdPtr);

     Channel->TailBdAddr = (UINTPTR)LastBdPtr;
     Channel->TailBdPhysAddr = XAXIVDMA_VIRT_TO_PHYS((UINTPTR)LastBdPtr);


     Channel->IsValid = 1;

     return;
 }
 /*****************************************************************************/
 /**
  * This function checks whether reset operation is done
  *
  * @param Channel is the pointer to the DMA channel to work on
  *
  * @return
  * - 0 if reset is done
  * - 1 if reset is still going
  *
  *****************************************************************************/
 int XAxiVdma_ChannelResetNotDone(XAxiVdma_Channel *Channel)
 {
     return (XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
             XAXIVDMA_CR_RESET_MASK);
 }
 /*****************************************************************************/
 /**
  * This function resets one DMA channel
  *
  * The registers will be default values after the reset
  *
  * @param Channel is the pointer to the DMA channel to work on
  *
  * @return
  *  None
  *
  *****************************************************************************/
 void XAxiVdma_ChannelReset(XAxiVdma_Channel *Channel)
 {
     XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
         XAXIVDMA_CR_RESET_MASK);

     return;
 }
 /*****************************************************************************/
 /*
  * Check whether a DMA channel is running
  *
  * @param Channel is the pointer to the channel to work on
  *
  * @return
  * - non zero if the channel is running
  * - 0 is the channel is idle
  *
  *****************************************************************************/
 int XAxiVdma_ChannelIsRunning(XAxiVdma_Channel *Channel)
 {
     u32 Bits;

     /* If halted bit set, channel is not running
      */
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
               XAXIVDMA_SR_HALTED_MASK;

     if (Bits) {
         return 0;
     }

     /* If Run/Stop bit low, then channel is not running
      */
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
               XAXIVDMA_CR_RUNSTOP_MASK;

     if (!Bits) {
         return 0;
     }

     return 1;
 }
 /*****************************************************************************/
 /**
  * Check whether a DMA channel is busy
  *
  * @param Channel is the pointer to the channel to work on
  *
  * @return
  * - non zero if the channel is busy
  * - 0 is the channel is idle
  *
  *****************************************************************************/
 int XAxiVdma_ChannelIsBusy(XAxiVdma_Channel *Channel)
 {
     u32 Bits;

     /* If the channel is idle, then it is not busy
      */
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
               XAXIVDMA_SR_IDLE_MASK;

     if (Bits) {
         return 0;
     }

     /* If the channel is halted, then it is not busy
      */
     Bits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
               XAXIVDMA_SR_HALTED_MASK;

     if (Bits) {
         return 0;
     }

     /* Otherwise, it is busy
      */
     return 1;
 }

//  /*****************************************************************************/
//  /*
//   * Check DMA channel errors
//   *
//   * @param Channel is the pointer to the channel to work on
//   *
//   * @return
//   *      Error bits of the channel, 0 means no errors
//   *
//   *****************************************************************************/
//  u32 XAxiVdma_ChannelErrors(XAxiVdma_Channel *Channel)
//  {
//      return (XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET)
//              & XAXIVDMA_SR_ERR_ALL_MASK);
//  }
//  /*****************************************************************************/
//  /*
//   * Clear DMA channel errors
//   *
//   * @param Channel is the pointer to the channel to work on
//   * @param ErrorMask is the mask of error bits to clear.
//   *
//   * @return
//   *      None
//   *
//   *****************************************************************************/
//  void XAxiVdma_ClearChannelErrors(XAxiVdma_Channel *Channel, u32 ErrorMask)
//  {
//      u32 SrBits;
//
//      /* Write on Clear bits */
//          SrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET)
//                          | ErrorMask;
//
//      XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET,
//          SrBits);
//
//      return;
//  }
//
//  /*****************************************************************************/
//  /**
//   * Get the current status of a channel
//   *
//   * @param Channel is the pointer to the channel to work on
//   *
//   * @return
//   * The status of the channel
//   *
//   *****************************************************************************/
//  u32 XAxiVdma_ChannelGetStatus(XAxiVdma_Channel *Channel)
//  {
//      return XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET);
//  }
//  /*****************************************************************************/
//  /**
//   * Set the channel to run in parking mode
//   *
//   * @param Channel is the pointer to the channel to work on
//   *
//   * @return
//   *   - XST_SUCCESS if everything is fine
//   *   - XST_FAILURE if hardware is not running
//   *
//   *****************************************************************************/
//  int XAxiVdma_ChannelStartParking(XAxiVdma_Channel *Channel)
//  {
//      u32 CrBits;
//
//      if (!XAxiVdma_ChannelIsRunning(Channel)) {
//          xdbg_printf(XDBG_DEBUG_ERROR,
//              "Channel is not running, cannot start park mode\r\n");
//
//          return XST_FAILURE;
//      }
//
//      CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
//                  ~XAXIVDMA_CR_TAIL_EN_MASK;
//
//      XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
//          CrBits);
//
//      return XST_SUCCESS;
//  }
//
//  /*****************************************************************************/
//  /**
//   * Set the channel to run in circular mode, exiting parking mode
//   *
//   * @param Channel is the pointer to the channel to work on
//   *
//   * @return
//   *   None
//   *
//   *****************************************************************************/
//  void XAxiVdma_ChannelStopParking(XAxiVdma_Channel *Channel)
//  {
//      u32 CrBits;
//
//      CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) |
//                  XAXIVDMA_CR_TAIL_EN_MASK;
//
//      XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
//          CrBits);
//
//      return;
//  }
//  /*****************************************************************************/
//  /**
//   * Set the channel to run in frame count enable mode
//   *
//   * @param Channel is the pointer to the channel to work on
//   *
//   * @return
//   *   None
//   *
//   *****************************************************************************/
//  void XAxiVdma_ChannelStartFrmCntEnable(XAxiVdma_Channel *Channel)
//  {
//      u32 CrBits;
//
//      CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) |
//                  XAXIVDMA_CR_FRMCNT_EN_MASK;
//
//      XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
//          CrBits);
//
//      return;
//  }
//
//  /*****************************************************************************/
//  /**
//   * Setup BD addresses to a different memory region
//   *
//   * In some systems, it is convenient to put BDs into a certain region of the
//   * memory. This function enables that.
//   *
//   * @param Channel is the pointer to the channel to work on
//   * @param BdAddrPhys is the physical starting address for BDs
//   * @param BdAddrVirt is the Virtual starting address for BDs. For systems that
//   *         do not use MMU, then virtual address is the same as physical address
//   *
//   * @return
//   * - XST_SUCCESS for a successful setup
//   * - XST_DEVICE_BUSY if the DMA channel is not idle, BDs are still being used
//   *
//   * @notes
//   * We assume that the memory region starting from BdAddrPhys is large enough
//   * to hold all the BDs.
//   *
//   *****************************************************************************/
//   int XAxiVdma_ChannelSetBdAddrs(XAxiVdma_Channel *Channel, UINTPTR BdAddrPhys,
//           UINTPTR BdAddrVirt)
//   {
//       int NumFrames = Channel->AllCnt;
//       int i;
//       UINTPTR NextPhys = BdAddrPhys;
//       UINTPTR CurrVirt = BdAddrVirt;
//
//       if (Channel->HasSG && XAxiVdma_ChannelIsBusy(Channel)) {
//           xdbg_printf(XDBG_DEBUG_ERROR,
//               "Channel is busy, cannot setup engine for transfer\r\n");
//
//           return XST_DEVICE_BUSY;
//       }
//
//       memset((void *)BdAddrPhys, 0, NumFrames * sizeof(XAxiVdma_Bd));
//       memset((void *)BdAddrVirt, 0, NumFrames * sizeof(XAxiVdma_Bd));
//
//       /* Set up the BD link list */
//       for (i = 0; i < NumFrames; i++) {
//           XAxiVdma_Bd *BdPtr;
//
//           BdPtr = (XAxiVdma_Bd *)CurrVirt;
//
//           /* The last BD connects to the first BD
//            */
//           if (i == (NumFrames - 1)) {
//               NextPhys = BdAddrPhys;
//           }
//           else {
//               NextPhys += sizeof(XAxiVdma_Bd);
//           }
//
//           XAxiVdma_BdSetNextPtr(BdPtr, NextPhys);
//           CurrVirt += sizeof(XAxiVdma_Bd);
//       }
//
//       /* Setup the BD addresses so that access the head/tail BDs fast
//        *
//        */
//       Channel->HeadBdPhysAddr = BdAddrPhys;
//       Channel->HeadBdAddr = BdAddrVirt;
//       Channel->TailBdPhysAddr = BdAddrPhys +
//                                 (NumFrames - 1) * sizeof(XAxiVdma_Bd);
//       Channel->TailBdAddr = BdAddrVirt +
//                                 (NumFrames - 1) * sizeof(XAxiVdma_Bd);
//
//       return XST_SUCCESS;
//   }
//   /*****************************************************************************/
//   /**
//    * Start a transfer
//    *
//    * This function setup the DMA engine and start the engine to do the transfer.
//    *
//    * @param Channel is the pointer to the channel to work on
//    * @param ChannelCfgPtr is the pointer to the setup structure
//    *
//    * @return
//    * - XST_SUCCESS for a successful submission
//    * - XST_FAILURE if channel has not being initialized
//    * - XST_DEVICE_BUSY if the DMA channel is not idle, BDs are still being used
//    * - XST_INVAID_PARAM if parameters in config structure not valid
//    *
//    *****************************************************************************/
//   int XAxiVdma_ChannelStartTransfer(XAxiVdma_Channel *Channel,
//       XAxiVdma_ChannelSetup *ChannelCfgPtr)
//   {
//       int Status;
//
//       if (!Channel->IsValid) {
//           xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");
//
//           return XST_FAILURE;
//       }
//
//       if (Channel->HasSG && XAxiVdma_ChannelIsBusy(Channel)) {
//           xdbg_printf(XDBG_DEBUG_ERROR,
//               "Channel is busy, cannot setup engine for transfer\r\n");
//
//           return XST_DEVICE_BUSY;
//       }
//
//       Status = XAxiVdma_ChannelConfig(Channel, ChannelCfgPtr);
//       if (Status != XST_SUCCESS) {
//           xdbg_printf(XDBG_DEBUG_ERROR,
//               "Channel config failed %d\r\n", Status);
//
//           return Status;
//       }
//
//       Status = XAxiVdma_ChannelSetBufferAddr(Channel,
//           ChannelCfgPtr->FrameStoreStartAddr, Channel->AllCnt);
//       if (Status != XST_SUCCESS) {
//           xdbg_printf(XDBG_DEBUG_ERROR,
//               "Channel setup buffer addr failed %d\r\n", Status);
//
//           return Status;
//       }
//
//       Status = XAxiVdma_ChannelStart(Channel);
//       if (Status != XST_SUCCESS) {
//           xdbg_printf(XDBG_DEBUG_ERROR,
//               "Channel start failed %d\r\n", Status);
//
//           return Status;
//       }
//
//       return XST_SUCCESS;
//   }
/*****************************************************************************/
/**
* Configure one DMA channel using the configuration structure
*
* Setup the control register and BDs, however, BD addresses are not set.
*
* @param Channel is the pointer to the channel to work on
* @param ChannelCfgPtr is the pointer to the setup structure
*
* @return
* - XST_SUCCESS if successful
* - XST_FAILURE if channel has not being initialized
* - XST_DEVICE_BUSY if the DMA channel is not idle
* - XST_INVALID_PARAM if fields in ChannelCfgPtr is not valid
*
*****************************************************************************/
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

   /* Check whether Hsize is properly aligned */
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

   /* Check whether Stride is properly aligned */
   if (ChannelCfgPtr->Stride < Channel->WordLength) {
       stride_align = (u32)Channel->WordLength;
   } else {
       stride_align = (u32)(ChannelCfgPtr->Stride % Channel->WordLength);
       if (stride_align > 0)
           stride_align = (Channel->WordLength - stride_align);
   }
   /* If hardware has no DRE, then Hsize and Stride must
    * be word-aligned
    */
   if (!Channel->HasDRE) {
       if (hsize_align != 0) {
           /* Adjust hsize to multiples of stream/mm data width*/
           ChannelCfgPtr->HoriSizeInput += hsize_align;
       }
       if (stride_align != 0) {
           /* Adjust stride to multiples of stream/mm data width*/
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
       /* Park mode */
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

   /* Write the control register value out
    */
   XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
       CrBits);

   if (Channel->HasSG) {
       /* Setup the information in BDs
        *
        * All information is available except the buffer addrs
        * Buffer addrs are set through XAxiVdma_ChannelSetBufferAddr()
        */
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
   else {   /* direct register mode */
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
/*****************************************************************************/
/**
 * Configure buffer addresses for one DMA channel
 *
 * The buffer addresses are physical addresses.
 * Access to 32 Frame Buffer Addresses in direct mode is done through
 * XAxiVdma_ChannelHiFrmAddrEnable/Disable Functions.
 * 0 - Access Bank0 Registers (0x5C - 0x98)
 * 1 - Access Bank1 Registers (0x5C - 0x98)
 *
 * @param Channel is the pointer to the channel to work on
 * @param BufferAddrSet is the set of addresses for the transfers
 * @param NumFrames is the number of frames to set the address
 *
 * @return
 * - XST_SUCCESS if successful
 * - XST_FAILURE if channel has not being initialized
 * - XST_DEVICE_BUSY if the DMA channel is not idle, BDs are still being used
 * - XST_INVAID_PARAM if buffer address not valid, for example, unaligned
 * address with no DRE built in the hardware
 *
 *****************************************************************************/
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
//  printf("@ %4d:%s NumFrames=%d\n",__LINE__, __FUNCTION__, NumFrames);
//  printf("@ %4d:%s Addr=%X\n",__LINE__, __FUNCTION__, BufferAddrSet[0]);

    if (!Channel->IsValid) {
        xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");

        return XST_FAILURE;
    }

    WordLenBits = (u32)(Channel->WordLength - 1);

    /* If hardware has no DRE, then buffer addresses must
     * be word-aligned
     */
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
                /* For a 40-bit address XAXIVDMA_MAX_FRAMESTORE
                 * value should be set to 16 */
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
/*****************************************************************************/
/**
 * Start one DMA channel
 *
 * @param Channel is the pointer to the channel to work on
 *
 * @return
 * - XST_SUCCESS if successful
 * - XST_FAILURE if channel is not initialized
 * - XST_DMA_ERROR if:
 *   . The DMA channel fails to stop
 *   . The DMA channel fails to start
 * - XST_DEVICE_BUSY is the channel is doing transfers
 *
 *****************************************************************************/
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
    }

    /* If channel is not running, setup the CDESC register and
     * set the channel to run
     */
    if (!XAxiVdma_ChannelIsRunning(Channel)) {

        if (Channel->HasSG) {
            /* Set up the current bd register
             *
             * Can only setup current bd register when channel is halted
             */
            XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CDESC_OFFSET,
                Channel->HeadBdPhysAddr);
        }

        /* Start DMA hardware
         */
        CrBits = XAxiVdma_ReadReg(Channel->ChanBase,
             XAXIVDMA_CR_OFFSET);

        CrBits = XAxiVdma_ReadReg(Channel->ChanBase,
             XAXIVDMA_CR_OFFSET) | XAXIVDMA_CR_RUNSTOP_MASK;

        XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
            CrBits);

    }
    if (XAxiVdma_ChannelIsRunning(Channel)) {

        /* Start DMA transfers
         *
         */

        if (Channel->HasSG) {
            /* SG mode:
             * Update the tail pointer so that hardware will start
             * fetching BDs
             */
            XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_TDESC_OFFSET,
               Channel->TailBdPhysAddr);
        }
        else {
            /* Direct register mode:
             * Update vsize to start the channel
             */
            XAxiVdma_WriteReg(Channel->StartAddrBase,
                XAXIVDMA_VSIZE_OFFSET, Channel->Vsize);

        }

        return XST_SUCCESS;
    }
    else {
        xdbg_printf(XDBG_DEBUG_ERROR,
            "Failed to start channel %x\r\n",
                (unsigned int)Channel->ChanBase);

        return XST_DMA_ERROR;
    }
}
/*****************************************************************************/
/**
 * Stop one DMA channel
 *
 * @param Channel is the pointer to the channel to work on
 *
 * @return
 *  None
 *
 *****************************************************************************/
void XAxiVdma_ChannelStop(XAxiVdma_Channel *Channel)
{
    u32 CrBits;

    if (!XAxiVdma_ChannelIsRunning(Channel)) {
        return;
    }

    /* Clear the RS bit in CR register
     */
    CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
        (~XAXIVDMA_CR_RUNSTOP_MASK);

    XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET, CrBits);

    return;
}
/*****************************************************************************/
/**
 * Dump registers from one DMA channel
 *
 * @param Channel is the pointer to the channel to work on
 *
 * @return
 *  None
 *
 *****************************************************************************/
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
// /*****************************************************************************/
// /**
//  * Set the frame counter and delay counter for one channel
//  *
//  * @param Channel is the pointer to the channel to work on
//  * @param FrmCnt is the frame counter value to be set
//  * @param DlyCnt is the delay counter value to be set
//  *
//  * @return
//  *   - XST_SUCCESS if setup finishes successfully
//  *   - XST_FAILURE if channel is not initialized
//  *   - XST_INVALID_PARAM if the configuration structure has invalid values
//  *   - XST_NO_FEATURE if Frame Counter or Delay Counter is disabled
//  *
//  *****************************************************************************/
// int XAxiVdma_ChannelSetFrmCnt(XAxiVdma_Channel *Channel, u8 FrmCnt, u8 DlyCnt)
// {
//     u32 CrBits;
//
//     if (!Channel->IsValid) {
//         xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");
//
//         return XST_FAILURE;
//     }
//
//     if (!FrmCnt) {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Frame counter value must be non-zero\r\n");
//
//         return XST_INVALID_PARAM;
//     }
//
//     CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
//         ~(XAXIVDMA_DELAY_MASK | XAXIVDMA_FRMCNT_MASK);
//
//     if (Channel->DbgFeatureFlags & XAXIVDMA_ENABLE_DBG_FRM_CNTR) {
//         CrBits |= (FrmCnt << XAXIVDMA_FRMCNT_SHIFT);
//     } else {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Channel Frame counter is disabled\r\n");
//         return XST_NO_FEATURE;
//     }
//     if (Channel->DbgFeatureFlags & XAXIVDMA_ENABLE_DBG_DLY_CNTR) {
//         CrBits |= (DlyCnt << XAXIVDMA_DELAY_SHIFT);
//     } else {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Channel Delay counter is disabled\r\n");
//         return XST_NO_FEATURE;
//     }
//
//     XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
//         CrBits);
//
//     return XST_SUCCESS;
// }
// /*****************************************************************************/
// /**
//  * Get the frame counter and delay counter for both channels
//  *
//  * @param Channel is the pointer to the channel to work on
//  * @param FrmCnt is the pointer for the returning frame counter value
//  * @param DlyCnt is the pointer for the returning delay counter value
//  *
//  * @return
//  *  None
//  *
//  * @note
//  *  If FrmCnt return as 0, then the channel is not initialized
//  *****************************************************************************/
// void XAxiVdma_ChannelGetFrmCnt(XAxiVdma_Channel *Channel, u8 *FrmCnt,
//         u8 *DlyCnt)
// {
//     u32 CrBits;
//
//     if (!Channel->IsValid) {
//         xdbg_printf(XDBG_DEBUG_ERROR, "Channel not initialized\r\n");
//
//         *FrmCnt = 0;
//         return;
//     }
//
//     CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET);
//
//     if (Channel->DbgFeatureFlags & XAXIVDMA_ENABLE_DBG_FRM_CNTR) {
//         *FrmCnt = (CrBits & XAXIVDMA_FRMCNT_MASK) >>
//                 XAXIVDMA_FRMCNT_SHIFT;
//     } else {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Channel Frame counter is disabled\r\n");
//     }
//     if (Channel->DbgFeatureFlags & XAXIVDMA_ENABLE_DBG_DLY_CNTR) {
//         *DlyCnt = (CrBits & XAXIVDMA_DELAY_MASK) >>
//                 XAXIVDMA_DELAY_SHIFT;
//     } else {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Channel Delay counter is disabled\r\n");
//     }
//
//
//     return;
// }
// /*****************************************************************************/
// /**
//  * Enable interrupts for a channel. Interrupts that are not specified by the
//  * interrupt mask are not affected.
//  *
//  * @param Channel is the pointer to the channel to work on
//  * @param IntrType is the interrupt mask for interrupts to be enabled
//  *
//  * @return
//  *  None.
//  *
//  *****************************************************************************/
// void XAxiVdma_ChannelEnableIntr(XAxiVdma_Channel *Channel, u32 IntrType)
// {
//     u32 CrBits;
//
//     if ((IntrType & XAXIVDMA_IXR_ALL_MASK) == 0) {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Enable intr with null intr mask value %x\r\n",
//             (unsigned int)IntrType);
//
//         return;
//     }
//
//     CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
//               ~XAXIVDMA_IXR_ALL_MASK;
//
//     CrBits |= IntrType & XAXIVDMA_IXR_ALL_MASK;
//
//     XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
//         CrBits);
//
//     return;
// }
// /*****************************************************************************/
// /**
//  * Disable interrupts for a channel. Interrupts that are not specified by the
//  * interrupt mask are not affected.
//  *
//  * @param Channel is the pointer to the channel to work on
//  * @param IntrType is the interrupt mask for interrupts to be disabled
//  *
//  * @return
//  *  None.
//  *
//  *****************************************************************************/
// void XAxiVdma_ChannelDisableIntr(XAxiVdma_Channel *Channel, u32 IntrType)
// {
//     u32 CrBits;
//     u32 IrqBits;
//
//     if ((IntrType & XAXIVDMA_IXR_ALL_MASK) == 0) {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Disable intr with null intr mask value %x\r\n",
//             (unsigned int)IntrType);
//
//         return;
//     }
//
//     CrBits = XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET);
//
//     IrqBits = (CrBits & XAXIVDMA_IXR_ALL_MASK) &
//                ~(IntrType & XAXIVDMA_IXR_ALL_MASK);
//
//     CrBits &= ~XAXIVDMA_IXR_ALL_MASK;
//
//     XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET,
//         CrBits | IrqBits);
//
//     return;
// }
//
// /*****************************************************************************/
// /**
//  * Get pending interrupts of a channel.
//  *
//  * @param Channel is the pointer to the channel to work on
//  *
//  * @return
//  *  The interrupts mask represents pending interrupts.
//  *
//  *****************************************************************************/
// u32 XAxiVdma_ChannelGetPendingIntr(XAxiVdma_Channel *Channel)
// {
//     return (XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET) &
//               XAXIVDMA_IXR_ALL_MASK);
// }
// /*****************************************************************************/
// /**
//  * Clear interrupts of a channel. Interrupts that are not specified by the
//  * interrupt mask are not affected.
//  *
//  * @param Channel is the pointer to the channel to work on
//  * @param IntrType is the interrupt mask for interrupts to be cleared
//  *
//  * @return
//  *  None.
//  *
//  *****************************************************************************/
// void XAxiVdma_ChannelIntrClear(XAxiVdma_Channel *Channel, u32 IntrType)
// {
//
//     if ((IntrType & XAXIVDMA_IXR_ALL_MASK) == 0) {
//         xdbg_printf(XDBG_DEBUG_ERROR,
//             "Clear intr with null intr mask value %x\r\n",
//             (unsigned int)IntrType);
//
//         return;
//     }
//
//     /* Only interrupts bits are writable in status register
//      */
//     XAxiVdma_WriteReg(Channel->ChanBase, XAXIVDMA_SR_OFFSET,
//         IntrType & XAXIVDMA_IXR_ALL_MASK);
//
//     return;
// }
//
// /*****************************************************************************/
// /**
//  * Get the enabled interrupts of a channel.
//  *
//  * @param Channel is the pointer to the channel to work on
//  *
//  * @return
//  *  The interrupts mask represents pending interrupts.
//  *
//  *****************************************************************************/
// u32 XAxiVdma_ChannelGetEnabledIntr(XAxiVdma_Channel *Channel)
// {
//     return (XAxiVdma_ReadReg(Channel->ChanBase, XAXIVDMA_CR_OFFSET) &
//               XAXIVDMA_IXR_ALL_MASK);
// }
/*********************** BD Functions ****************************************/
/*****************************************************************************/
/*
 * Read one word from BD
 *
 * @param BdPtr is the BD to work on
 * @param Offset is the byte offset to read from
 *
 * @return
 *  The word value
 *
 *****************************************************************************/
static u32 XAxiVdma_BdRead(XAxiVdma_Bd *BdPtr, int Offset)
{
    DBG("%s: RD %08X\n", __FUNCTION__, (uint32_t)BdPtr+Offset);
    return (*(u32 *)((UINTPTR)(void *)BdPtr + Offset));
}

/*****************************************************************************/
/*
 * Set one word in BD
 *
 * @param BdPtr is the BD to work on
 * @param Offset is the byte offset to write to
 * @param Value is the value to write to the BD
 *
 * @return
 *  None
 *
 *****************************************************************************/
static void XAxiVdma_BdWrite(XAxiVdma_Bd *BdPtr, int Offset, u32 Value)
{
    // DBG("%s: WR %08X <- %08X\n", __FUNCTION__, (uint32_t)BdPtr+Offset, Value);
    // *(u32 *)((UINTPTR)(void *)BdPtr + Offset) = Value;

    return;
}

/*****************************************************************************/
/*
 * Set the next ptr from BD
 *
 * @param BdPtr is the BD to work on
 * @param NextPtr is the next ptr to set in BD
 *
 * @return
 *  None
 *
 *****************************************************************************/
static void XAxiVdma_BdSetNextPtr(XAxiVdma_Bd *BdPtr, u32 NextPtr)
{
    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_NDESC_OFFSET, NextPtr);
    return;
}
/*****************************************************************************/
/*
 * Set the start address from BD
 *
 * The address is physical address.
 *
 * @param BdPtr is the BD to work on
 * @param Addr is the address to set in BD
 *
 * @return
 *  None
 *
 *****************************************************************************/
static void XAxiVdma_BdSetAddr(XAxiVdma_Bd *BdPtr, u32 Addr)
{
    XAxiVdma_BdWrite(BdPtr, XAXIVDMA_BD_START_ADDR_OFFSET, Addr);

    return;
}

/*****************************************************************************/
/*
 * Set the vertical size for a BD
 *
 * @param BdPtr is the BD to work on
 * @param Vsize is the vertical size to set in BD
 *
 * @return
 *  - XST_SUCCESS if successful
 *  - XST_INVALID_PARAM if argument Vsize is invalid
 *
 *****************************************************************************/
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
/*****************************************************************************/
/*
 * Set the horizontal size for a BD
 *
 * @param BdPtr is the BD to work on
 * @param Hsize is the horizontal size to set in BD
 *
 * @return
 *  - XST_SUCCESS if successful
 *  - XST_INVALID_PARAM if argument Hsize is invalid
 *
 *****************************************************************************/
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

/*****************************************************************************/
/*
 * Set the stride size for a BD
 *
 * @param BdPtr is the BD to work on
 * @param Stride is the stride size to set in BD
 *
 * @return
 *  - XST_SUCCESS if successful
 *  - XST_INVALID_PARAM if argument Stride is invalid
 *
 *****************************************************************************/
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
/*****************************************************************************/
/*
 * Set the frame delay for a BD
 *
 * @param BdPtr is the BD to work on
 * @param FrmDly is the frame delay value to set in BD
 *
 * @return
 *  - XST_SUCCESS if successful
 *  - XST_INVALID_PARAM if argument FrmDly is invalid
 *
 *****************************************************************************/
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



XAxiVdma_Config XAxiVdma_ConfigTable[XPAR_XAXIVDMA_NUM_INSTANCES] =
{
    {
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_DEVICE_ID,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_BASEADDR,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_NUM_FSTORES,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_MM2S,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_MM2S_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_M_AXI_MM2S_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_S2MM,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_S2MM_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_M_AXI_S2MM_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_SG,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_VIDPRMTR_READS,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_USE_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_FLUSH_ON_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_MM2S_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_MM2S_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_INTERNAL_GENLOCK,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_SOF_ENABLE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_M_AXIS_MM2S_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S_AXIS_S2MM_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_1,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_5,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_6,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_7,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_9,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_13,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_14,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_15,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_ALL,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ADDR_WIDTH
    },
    {
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_DEVICE_ID,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_BASEADDR,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_NUM_FSTORES,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_INCLUDE_MM2S,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_INCLUDE_MM2S_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_M_AXI_MM2S_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_INCLUDE_S2MM,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_INCLUDE_S2MM_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_M_AXI_S2MM_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_INCLUDE_SG,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_VIDPRMTR_READS,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_USE_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_FLUSH_ON_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_MM2S_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_S2MM_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_MM2S_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_S2MM_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_INCLUDE_INTERNAL_GENLOCK,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_S2MM_SOF_ENABLE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_M_AXIS_MM2S_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_S_AXIS_S2MM_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_1,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_5,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_6,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_7,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_9,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_13,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_14,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_INFO_15,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ENABLE_DEBUG_ALL,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_1_ADDR_WIDTH
    },
    {
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_DEVICE_ID,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_BASEADDR,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_NUM_FSTORES,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_INCLUDE_MM2S,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_INCLUDE_MM2S_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_M_AXI_MM2S_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_INCLUDE_S2MM,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_INCLUDE_S2MM_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_M_AXI_S2MM_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_INCLUDE_SG,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_VIDPRMTR_READS,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_USE_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_FLUSH_ON_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_MM2S_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_S2MM_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_MM2S_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_S2MM_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_INCLUDE_INTERNAL_GENLOCK,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_S2MM_SOF_ENABLE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_M_AXIS_MM2S_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_S_AXIS_S2MM_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_1,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_5,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_6,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_7,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_9,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_13,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_14,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_INFO_15,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ENABLE_DEBUG_ALL,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_2_ADDR_WIDTH
    },
    {
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_DEVICE_ID,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_BASEADDR,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_NUM_FSTORES,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_INCLUDE_MM2S,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_INCLUDE_MM2S_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_M_AXI_MM2S_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_INCLUDE_S2MM,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_INCLUDE_S2MM_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_M_AXI_S2MM_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_INCLUDE_SG,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_VIDPRMTR_READS,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_USE_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_FLUSH_ON_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_MM2S_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_S2MM_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_MM2S_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_S2MM_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_INCLUDE_INTERNAL_GENLOCK,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_S2MM_SOF_ENABLE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_M_AXIS_MM2S_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_S_AXIS_S2MM_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_1,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_5,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_6,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_7,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_9,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_13,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_14,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_INFO_15,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ENABLE_DEBUG_ALL,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_3_ADDR_WIDTH
    },
    {
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_DEVICE_ID,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_BASEADDR,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_NUM_FSTORES,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_INCLUDE_MM2S,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_INCLUDE_MM2S_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_M_AXI_MM2S_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_INCLUDE_S2MM,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_INCLUDE_S2MM_DRE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_M_AXI_S2MM_DATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_INCLUDE_SG,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_VIDPRMTR_READS,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_USE_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_FLUSH_ON_FSYNC,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_MM2S_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_S2MM_LINEBUFFER_DEPTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_MM2S_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_S2MM_GENLOCK_MODE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_INCLUDE_INTERNAL_GENLOCK,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_S2MM_SOF_ENABLE,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_M_AXIS_MM2S_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_S_AXIS_S2MM_TDATA_WIDTH,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_1,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_5,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_6,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_7,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_9,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_13,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_14,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_INFO_15,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ENABLE_DEBUG_ALL,
        XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_4_ADDR_WIDTH
    }
};

#define INITIALIZATION_POLLING   100000

/*****************************************************************************/
/**
 * Get a channel
 *
 * @param InstancePtr is the DMA engine to work on
 * @param Direction is the direction for the channel to get
 *
 * @return
 * The pointer to the channel. Upon error, return NULL.
 *
 * @note
 * Since this function is internally used, we assume Direction is valid
 *****************************************************************************/
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

static int XAxiVdma_Major(XAxiVdma *InstancePtr) {
    u32 Reg;

    Reg = XAxiVdma_ReadReg(InstancePtr->BaseAddr, XAXIVDMA_VERSION_OFFSET);

    return (int)((Reg & XAXIVDMA_VERSION_MAJOR_MASK) >>
              XAXIVDMA_VERSION_MAJOR_SHIFT);
}

/*****************************************************************************/
/**
 * Initialize the driver with hardware configuration
 *
 * @param InstancePtr is the pointer to the DMA engine to work on
 * @param CfgPtr is the pointer to the hardware configuration structure
 * @param EffectiveAddr is the virtual address map for the device
 *
 * @return
 *  - XST_SUCCESS if everything goes fine
 *  - XST_FAILURE if reset the hardware failed, need system reset to recover
 *
 * @note
 * If channel fails reset,  then it will be set as invalid
 *****************************************************************************/
int XAxiVdma_CfgInitialize(XAxiVdma *InstancePtr, XAxiVdma_Config *CfgPtr,
				UINTPTR EffectiveAddr)
{
	XAxiVdma_Channel *RdChannel;
	XAxiVdma_Channel *WrChannel;
	int Polls;

	// /* Validate parameters */
	// Xil_AssertNonvoid(InstancePtr != NULL);
	// Xil_AssertNonvoid(CfgPtr != NULL);

	/* Initially, no interrupt callback functions
	 */
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

	/* The channels are not valid until being initialized
	 */
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

		/* Flush on Sync */
		RdChannel->FlushonFsync = CfgPtr->FlushonFsync;

		/* Dynamic Line Buffers Depth */
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

		/* Internal GenLock */
		RdChannel->GenLock = CfgPtr->Mm2SGenLock;

		/* Debug Info Parameter flags */
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

		/* At time of initialization, no transfers are going on,
		 * reset is expected to be quick
		 */
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

		/* Flush on Sync */
		WrChannel->FlushonFsync = CfgPtr->FlushonFsync;

		/* Dynamic Line Buffers Depth */
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

		/* Internal GenLock */
		WrChannel->GenLock = CfgPtr->S2MmGenLock;

		/* Frame Sync Source Selection*/
		WrChannel->S2MmSOF = CfgPtr->S2MmSOF;

		/* Debug Info Parameter flags */
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

		/* At time of initialization, no transfers are going on,
		 * reset is expected to be quick
		 */
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

/*****************************************************************************/
/**
 * Configure one DMA channel using the configuration structure
 *
 * @param InstancePtr is the pointer to the DMA engine to work on
 * @param Direction is the DMA channel to work on
 * @param DmaConfigPtr is the pointer to the setup structure
 *
 * @return
 * - XST_SUCCESS if successful
 * - XST_DEVICE_BUSY if the DMA channel is not idle, BDs are still being used
 * - XST_INVAID_PARAM if buffer address not valid, for example, unaligned
 *   address with no DRE built in the hardware, or Direction invalid
 * - XST_DEVICE_NOT_FOUND if the channel is invalid
 *
 *****************************************************************************/
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
/*****************************************************************************/
/**
 * Configure buffer addresses for one DMA channel
 *
 * @param InstancePtr is the pointer to the DMA engine to work on
 * @param Direction is the DMA channel to work on
 * @param BufferAddrSet is the set of addresses for the transfers
 *
 * @return
 * - XST_SUCCESS if successful
 * - XST_DEVICE_BUSY if the DMA channel is not idle, BDs are still being used
 * - XST_INVAID_PARAM if buffer address not valid, for example, unaligned
 *   address with no DRE built in the hardware, or Direction invalid
 * - XST_DEVICE_NOT_FOUND if the channel is invalid
 *
 *****************************************************************************/
int XAxiVdma_DmaSetBufferAddr(XAxiVdma *InstancePtr, u16 Direction,
        UINTPTR *BufferAddrSet)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel) {
        return XST_INVALID_PARAM;
    }

    if (Channel->IsValid) {
        return XAxiVdma_ChannelSetBufferAddr(Channel, BufferAddrSet,
            Channel->NumFrames);
    }
    else {
        return XST_DEVICE_NOT_FOUND;
    }
}
/*****************************************************************************/
/**
 * Start one DMA channel
 *
 * @param InstancePtr is the pointer to the DMA engine to work on
 * @param Direction is the DMA channel to work on
 *
 * @return
 * - XST_SUCCESS if channel started successfully
 * - XST_FAILURE otherwise
 * - XST_DEVICE_NOT_FOUND if the channel is invalid
 * - XST_INVALID_PARAM if Direction invalid
 *
 *****************************************************************************/
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
/*****************************************************************************/
/**
 * Stop one DMA channel
 *
 * @param InstancePtr is the pointer to the DMA engine to work on
 * @param Direction is the DMA channel to work on
 *
 * @return
 *  None
 *
 * @note
 * If channel is invalid, then do nothing on that channel
 *****************************************************************************/
void XAxiVdma_DmaStop(XAxiVdma *InstancePtr, u16 Direction)
{
    XAxiVdma_Channel *Channel;

    Channel = XAxiVdma_GetChannel(InstancePtr, Direction);

    if (!Channel) {
        return;
    }

    if (Channel->IsValid) {
        XAxiVdma_ChannelStop(Channel);
    }

    return;
}
/*****************************************************************************/
/**
 * Dump registers of one DMA channel
 *
 * @param InstancePtr is the pointer to the DMA engine to work on
 * @param Direction is the DMA channel to work on
 *
 * @return
 *  None
 *
 * @note
 * If channel is invalid, then do nothing on that channel
 *****************************************************************************/
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
//--------------------------------------------------------------------------
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
//--------------------------------------------------------------------------
void VDMA_Check_Errors(u32 addr)
{
	u32 *ptr;

	u32 outErrors;

	ptr = (volatile u32 *)(addr + XAXIVDMA_TX_OFFSET + XAXIVDMA_SR_OFFSET);

	outErrors = *ptr & 0x000046F0;

	xil_printf("========== %x =============\r\n", addr);
	if ( outErrors & 0x00004000 )
	{
		xil_printf( "\tMM2S_DMASR - ErrIrq\n\r" );
	}
	if ( outErrors & 0x00000400 )
	{
		xil_printf( "\tMM2S_DMASR - SGDecErr\n\r" );
	}
	if ( outErrors & 0x00000200 )
	{
		xil_printf( "\tMM2S_DMASR - SGSlvErr\n\r" );
	}
	if ( outErrors & 0x00000080 )
	{
		xil_printf( "\tMM2S_DMASR - SOFEarlyErr\n\r" );
	}
	if ( outErrors & 0x00000040 )
	{
		xil_printf( "\tMM2S_DMASR - DMADecErr\n\r" );
	}
	if ( outErrors & 0x00000020 )
	{
		xil_printf( "\tMM2S_DMASR - DMASlvErr\n\r" );
	}
	if ( outErrors & 0x00000010 )
	{
		xil_printf( "\tMM2S_DMASR - DMAIntErr\n\r" );
	}
	if (( outErrors & 0x00000000 ) == 0)
		xil_printf("No Error\r\n");
	else
		xil_printf("Error\r\n");
	xil_printf("=======================\r\n");
}
//--------------------------------------------------------------------------
void VDMA_Control_Dump(u32 addr)
{
	u32 *ptr;
	u32 i;
/*
	ptr = (u32 *)(addr);

	xil_printf("---------------------------------------------\r\n");
	for(i = 0; i < 0xFF; i += 4)
	{
		xil_printf("%x Address = %x\r\n", addr + i, *ptr);
		ptr++;
	}
	xil_printf("---------------------------------------------\r\n");
*/
}

int activate_vdma_0(int base, int hsize, int vsize, uint32_t *fb_mem)
{
    int status;
    XAxiVdma_Config *Config;

	xil_printf("VDMA0 Init Start\n");

    Config = XAxiVdma_LookupConfig(0);
    if (!Config) {
        xil_printf("No video DMA0 found\r\n");
        return XST_FAILURE;
    }

    status = XAxiVdma_CfgInitialize(&InstancePtr0, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }

    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);
//  printf("hsize: %d  Width: %d\n", hsize, Config->Mm2SStreamWidth);

    /* ************************************************** */
    /*           Setup the read channel                   */
    /*                                                    */
    /* ************************************************** */
    ReadCfg0.VertSizeInput       = vsize;
    ReadCfg0.HoriSizeInput       = stride;
    ReadCfg0.Stride              = stride;
    ReadCfg0.FrameDelay          = 0;      /* This example does not test frame delay */
    ReadCfg0.EnableCircularBuf   = 1;      /* Only 1 buffer, continuous loop */
    ReadCfg0.EnableSync          = 0;      /* Gen-Lock */
    ReadCfg0.PointNum            = 0;
    ReadCfg0.EnableFrameCounter  = 0;      /* Endless transfers */
    ReadCfg0.FixedFrameStoreAddr = 0;      /* We are not doing parking */

    status = XAxiVdma_DmaConfig(&InstancePtr0, XAXIVDMA_READ, &ReadCfg0);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

    /* Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer */
    status = XAxiVdma_DmaSetBufferAddr(&InstancePtr0, XAXIVDMA_READ, (UINTPTR *)fb_mem);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }
    /************* Read channel setup done ************** */

    /* ************************************************** */
    /*  Start the DMA engine (read channel) to transfer   */
    /*                                                    */
    /* ************************************************** */

    /* Start the Read channel of DMA Engine */
    DBG("Start VDMA\n");
    status = XAxiVdma_DmaStart(&InstancePtr0, XAXIVDMA_READ);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    /* ************ DMA engine start done *************** */
	xil_printf("VDMA0 Engine Start done\n");
    return XST_SUCCESS;
}

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
		xil_printf("");
	}

    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);
//  printf("hsize: %d  Width: %d\n", hsize, Config->Mm2SStreamWidth);

    /* ************************************************** */
    /*           Setup the read channel                   */
    /*                                                    */
    /* ************************************************** */
    ReadCfg1.VertSizeInput       = vsize;
    ReadCfg1.HoriSizeInput       = stride;
    ReadCfg1.Stride              = stride;
    ReadCfg1.FrameDelay          = 0;      /* This example does not test frame delay */
    ReadCfg1.EnableCircularBuf   = 1;      /* Only 1 buffer, continuous loop */
    ReadCfg1.EnableSync          = 0;      /* Gen-Lock */
    ReadCfg1.PointNum            = 0;
    ReadCfg1.EnableFrameCounter  = 0;      /* Endless transfers */
    ReadCfg1.FixedFrameStoreAddr = 0;      /* We are not doing parking */

    status = XAxiVdma_DmaConfig(&InstancePtr1, XAXIVDMA_READ, &ReadCfg1);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

	storage_offset = ReadCfg1.Stride;
	Addr = VDMA1_BASE_MEM  + storage_offset;
	ReadCfg1.FrameStoreStartAddr[0] = Addr;


    /* Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer */
    // status = XAxiVdma_DmaSetBufferAddr(&InstancePtr1, XAXIVDMA_READ, (UINTPTR *)vdma1_base);
	status = XAxiVdma_DmaSetBufferAddr(&InstancePtr1, XAXIVDMA_READ, ReadCfg1.FrameStoreStartAddr);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }
    /************* Read channel setup done ************** */

    /* ************************************************** */
    /*  Start the DMA engine (read channel) to transfer   */
    /*                                                    */
    /* ************************************************** */

    /* Start the Read channel of DMA Engine */
    DBG("Start VDMA1 \n");
    status = XAxiVdma_DmaStart(&InstancePtr1, XAXIVDMA_READ);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    /* ************ DMA engine start done *************** */
	VDMA_Config_Dump(Config);
	VDMA_Setup_Dump(&ReadCfg1);
	VDMA_Control_Dump(0x83010000);

	xil_printf("VDMA1 Engine Start done\n");
    return XST_SUCCESS;
}

int activate_vdma_2(int base, int hsize, int vsize, uint32_t *vdma2_base)
{
    int status;
    XAxiVdma_Config *Config;
	u32 Addr;
	u32 storage_offset;

	xil_printf("VDMA2 Init Start\n");

    Config = XAxiVdma_LookupConfig(2);
    if (!Config) {
        xil_printf("No video DMA2 found\r\n");
        return XST_FAILURE;
    }

    status = XAxiVdma_CfgInitialize(&InstancePtr2, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }

    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);
//  printf("hsize: %d  Width: %d\n", hsize, Config->Mm2SStreamWidth);

    /* ************************************************** */
    /*           Setup the read channel                   */
    /*                                                    */
    /* ************************************************** */
    ReadCfg2.VertSizeInput       = vsize;
    ReadCfg2.HoriSizeInput       = stride;
    ReadCfg2.Stride              = stride;
    ReadCfg2.FrameDelay          = 0;      /* This example does not test frame delay */
    ReadCfg2.EnableCircularBuf   = 1;      /* Only 1 buffer, continuous loop */
    ReadCfg2.EnableSync          = 0;      /* Gen-Lock */
    ReadCfg2.PointNum            = 0;
    ReadCfg2.EnableFrameCounter  = 0;      /* Endless transfers */
    ReadCfg2.FixedFrameStoreAddr = 0;      /* We are not doing parking */

    status = XAxiVdma_DmaConfig(&InstancePtr2, XAXIVDMA_READ, &ReadCfg2);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

	storage_offset = ReadCfg2.Stride;
	Addr = VDMA2_BASE_MEM; // + storage_offset;

	ReadCfg2.FrameStoreStartAddr[0] = Addr;


    /* Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer */
   //  status = XAxiVdma_DmaSetBufferAddr(&InstancePtr2, XAXIVDMA_READ, (UINTPTR *)vdma2_base);
	status = XAxiVdma_DmaSetBufferAddr(&InstancePtr2, XAXIVDMA_READ, ReadCfg2.FrameStoreStartAddr);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }
    /************* Read channel setup done ************** */

    /* ************************************************** */
    /*  Start the DMA engine (read channel) to transfer   */
    /*                                                    */
    /* ************************************************** */

    /* Start the Read channel of DMA Engine */
    DBG("Start VDMA2\n");
    status = XAxiVdma_DmaStart(&InstancePtr2, XAXIVDMA_READ);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    /* ************ DMA engine start done *************** */

	xil_printf("VDMA2 Engine Start done\n");
    return XST_SUCCESS;
}


int activate_vdma_3(int base, int hsize, int vsize, uint32_t *fb_mem)
{
    int status;
    XAxiVdma_Config *Config;

	xil_printf("VDMA3 Init Start\n");
    Config = XAxiVdma_LookupConfig(3);
    if (!Config) {
        xil_printf("No video DMA3 found\r\n");
        return XST_FAILURE;
    }else
	{
		xil_printf("Video DMA3 fount\n", Config->BaseAddress);
	}

    status = XAxiVdma_CfgInitialize(&InstancePtr3, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }

    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);
//  printf("hsize: %d  Width: %d\n", hsize, Config->Mm2SStreamWidth);

    /* ************************************************** */
    /*           Setup the read channel                   */
    /*                                                    */
    /* ************************************************** */
    ReadCfg3.VertSizeInput       = vsize;
    ReadCfg3.HoriSizeInput       = stride;
    ReadCfg3.Stride              = stride;
    ReadCfg3.FrameDelay          = 0;      /* This example does not test frame delay */
    ReadCfg3.EnableCircularBuf   = 1;      /* Only 1 buffer, continuous loop */
    ReadCfg3.EnableSync          = 0;      /* Gen-Lock */
    ReadCfg3.PointNum            = 0;
    ReadCfg3.EnableFrameCounter  = 0;      /* Endless transfers */
    ReadCfg3.FixedFrameStoreAddr = 0;      /* We are not doing parking */

    status = XAxiVdma_DmaConfig(&InstancePtr3, XAXIVDMA_READ, &ReadCfg3);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

    /* Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer */
    status = XAxiVdma_DmaSetBufferAddr(&InstancePtr3, XAXIVDMA_READ, (UINTPTR *)fb_mem);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }
    /************* Read channel setup done ************** */

    /* ************************************************** */
    /*  Start the DMA engine (read channel) to transfer   */
    /*                                                    */
    /* ************************************************** */

    /* Start the Read channel of DMA Engine */
    DBG("Start VDMA3\n");
    status = XAxiVdma_DmaStart(&InstancePtr3, XAXIVDMA_READ);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    /* ************ DMA engine start done *************** */
	xil_printf("VDMA3 Engine Start done\n");
	VDMA_Config_Dump(Config);
	VDMA_Setup_Dump(&ReadCfg3);
	VDMA_Control_Dump(0x83030000);
    return XST_SUCCESS;
}


int activate_vdma_4(int base, int hsize, int vsize, uint32_t *vdma4_base, int Mode)
{
    int status;
    XAxiVdma_Config *Config;
	u32 Addr;
	u32 storage_offset;

	xil_printf("VDMA4 Init Start\n");
    Config = XAxiVdma_LookupConfig(4);
    if (!Config) {
        xil_printf("No video DMA4 found\r\n");
        return XST_FAILURE;
    }else
	{
		xil_printf("VDMA4 Config ok\n");
	}

    status = XAxiVdma_CfgInitialize(&InstancePtr4, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("Configuration Initialization failed, status: 0x%X\r\n", status);
        return status;
    }

    u32 stride = hsize * (Config->Mm2SStreamWidth>>3);
//  printf("hsize: %d  Width: %d\n", hsize, Config->Mm2SStreamWidth);

    /* ************************************************** */
    /*           Setup the read channel                   */
    /*                                                    */
    /* ************************************************** */
    ReadCfg4.VertSizeInput       = vsize;
    ReadCfg4.HoriSizeInput       = stride;
    ReadCfg4.Stride              = stride;
    ReadCfg4.FrameDelay          = 0;      /* This example does not test frame delay */
    ReadCfg4.EnableCircularBuf   = 1;      /* Only 1 buffer, continuous loop */
    ReadCfg4.EnableSync          = 0;      /* Gen-Lock */
    ReadCfg4.PointNum            = 0;
    ReadCfg4.EnableFrameCounter  = 0;      /* Endless transfers */
    ReadCfg4.FixedFrameStoreAddr = 0;      /* We are not doing parking */

    status = XAxiVdma_DmaConfig(&InstancePtr4, XAXIVDMA_WRITE, &ReadCfg4);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel config failed, status: 0x%X\r\n", status);
        return status;
    }

	storage_offset = ReadCfg4.Stride;
	if(Mode == 0)			// FULL HD
		Addr = VDMA4_BASE_MEM; // + storage_offset + ((1920 * 180 * 4) + (320 *4));
	else 					// HD
		Addr = VDMA4_BASE_MEM + storage_offset + ((1920 * 180 * 4) + (320 *4));
	ReadCfg4.FrameStoreStartAddr[0] = Addr;


    /* Set the buffer addresses for transfer in the DMA engine. This is address first pixel of the framebuffer */
    // status = XAxiVdma_DmaSetBufferAddr(&InstancePtr4, XAXIVDMA_WRITE, (UINTPTR *)vdma4_base);
	status = XAxiVdma_DmaSetBufferAddr(&InstancePtr4, XAXIVDMA_WRITE, ReadCfg4.FrameStoreStartAddr);
    if (status != XST_SUCCESS) {
        xil_printf("Read channel set buffer address failed, status: 0x%X\r\n", status);
        return status;
    }
    /************* Read channel setup done ************** */

    /* ************************************************** */
    /*  Start the DMA engine (read channel) to transfer   */
    /*                                                    */
    /* ************************************************** */

    /* Start the Read channel of DMA Engine */
    DBG("Start VDMA4\n");
    status = XAxiVdma_DmaStart(&InstancePtr4, XAXIVDMA_WRITE);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start DMA engine (read channel), status: 0x%X\r\n", status);
        return status;
    }
    /* ************ DMA engine start done *************** */

	VDMA_Config_Dump(Config);
	VDMA_Setup_Dump(&ReadCfg4);
	xil_printf("VDMA4 Engine Start done  ... \n");
    return XST_SUCCESS;
}
