#include <inttypes.h>
#include "xaxivdma_hw.h"
#include "xdebug.h"

#ifndef __XILTYPES_H__
#define __XILTYPES_H__

#define xil_printf  printf

typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uintptr_t UINTPTR;

/**
 * @brief  Returns 32-63 bits of a number.
 * @param  n : Number being accessed.
 * @return Bits 32-63 of number.
 *
 * @note    A basic shift-right of a 64- or 32-bit quantity.
 *          Use this to suppress the "right shift count >= width of type"
 *          warning when that quantity is 32-bits.
 */
#define UPPER_32_BITS(n) ((u32)(((n) >> 16) >> 16))

/**
 * @brief  Returns 0-31 bits of a number
 * @param  n : Number being accessed.
 * @return Bits 0-31 of number
 */
#define LOWER_32_BITS(n) ((u32)(n))


#define XST_SUCCESS                     0L
#define XST_FAILURE                     1L
#define XST_DEVICE_NOT_FOUND            2L
#define XST_DEVICE_BLOCK_NOT_FOUND      3L
#define XST_INVALID_VERSION             4L
#define XST_DEVICE_IS_STARTED           5L
#define XST_DEVICE_IS_STOPPED           6L
#define XST_FIFO_ERROR                  7L  /*!< An error occurred during an
                           operation with a FIFO such as
                           an underrun or overrun, this
                           error requires the device to
                           be reset */
#define XST_RESET_ERROR                 8L  /*!< An error occurred which requires
                           the device to be reset */
#define XST_DMA_ERROR                   9L  /*!< A DMA error occurred, this error
                           typically requires the device
                           using the DMA to be reset */
#define XST_NOT_POLLED                  10L /*!< The device is not configured for
                           polled mode operation */
#define XST_FIFO_NO_ROOM                11L /*!< A FIFO did not have room to put
                           the specified data into */
#define XST_BUFFER_TOO_SMALL            12L /*!< The buffer is not large enough
                           to hold the expected data */
#define XST_NO_DATA                     13L /*!< There was no data available */
#define XST_REGISTER_ERROR              14L /*!< A register did not contain the
                           expected value */
#define XST_INVALID_PARAM               15L /*!< An invalid parameter was passed
                           into the function */
#define XST_NOT_SGDMA                   16L /*!< The device is not configured for
                           scatter-gather DMA operation */
#define XST_LOOPBACK_ERROR              17L /*!< A loopback test failed */
#define XST_NO_CALLBACK                 18L /*!< A callback has not yet been
                           registered */
#define XST_NO_FEATURE                  19L /*!< Device is not configured with
                           the requested feature */
#define XST_NOT_INTERRUPT               20L /*!< Device is not configured for
                           interrupt mode operation */
#define XST_DEVICE_BUSY                 21L /*!< Device is busy */
#define XST_ERROR_COUNT_MAX             22L /*!< The error counters of a device
                           have maxed out */
#define XST_IS_STARTED                  23L /*!< Used when part of device is
                           already started i.e.
                           sub channel */
#define XST_IS_STOPPED                  24L /*!< Used when part of device is
                           already stopped i.e.
                           sub channel */
#define XST_DATA_LOST                   26L /*!< Driver defined error */
#define XST_RECV_ERROR                  27L /*!< Generic receive error */
#define XST_SEND_ERROR                  28L /*!< Generic transmit error */
#define XST_NOT_ENABLED                 29L /*!< A requested service is not
                           available because it has not
                           been enabled */


/* Definitions for driver AXIVDMA */
#define XPAR_XAXIVDMA_NUM_INSTANCES 1U

/* Definitions for peripheral VIDEO_SUBSYSTEM_AXI_VDMA_0 */
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_DEVICE_ID 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_BASEADDR 0x000000U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_HIGHADDR 0x00FFFFU
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_NUM_FSTORES 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_MM2S 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_MM2S_DRE 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_M_AXI_MM2S_DATA_WIDTH 32U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_S2MM 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_S2MM_DRE 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_M_AXI_S2MM_DATA_WIDTH 64U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_AXI_MM2S_ACLK_FREQ_HZ 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_AXI_S2MM_ACLK_FREQ_HZ 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_MM2S_GENLOCK_MODE 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_MM2S_GENLOCK_NUM_MASTERS 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_GENLOCK_MODE 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_GENLOCK_NUM_MASTERS 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_SG 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_VIDPRMTR_READS 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_USE_FSYNC 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_FLUSH_ON_FSYNC 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_MM2S_LINEBUFFER_DEPTH 512U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_LINEBUFFER_DEPTH 512U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_INCLUDE_INTERNAL_GENLOCK 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S2MM_SOF_ENABLE 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_M_AXIS_MM2S_TDATA_WIDTH 32U // RGBA
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_S_AXIS_S2MM_TDATA_WIDTH 32U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_1 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_5 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_6 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_7 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_9 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_13 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_14 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_INFO_15 1U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ENABLE_DEBUG_ALL 0U
#define XPAR_VIDEO_SUBSYSTEM_AXI_VDMA_0_ADDR_WIDTH 32U

/** @name Buffer Descriptor Alignment
 *  @{
 */
#define XAXIVDMA_BD_MINIMUM_ALIGNMENT    0x20  /**< Minimum byte alignment
                                               requirement for descriptors */
#define XAXIVDMA_BD_MINIMUM_ALIGNMENT_WD 0x8  /**< Minimum word alignment
                                               requirement for descriptors */

/**
* Maximum number of the frame store
*/
#define XAXIVDMA_MAX_FRAMESTORE    32  /**< Maximum # of the frame store */

#define XAXIVDMA_MAX_FRAMESTORE_64 16  /**< Maximum # of the frame store for 64 bit*/



/* Buffer Descriptor (BD) is only visible in this file
 */
typedef u32 XAxiVdma_Bd[XAXIVDMA_BD_MINIMUM_ALIGNMENT_WD];

/* The DMA channel is only visible to driver files
 */
typedef struct {
    UINTPTR ChanBase;       /* Base address for this channel */
    UINTPTR InstanceBase;   /* Base address for the whole device */
    UINTPTR StartAddrBase;  /* Start address register array base */

    int IsValid;        /* Whether the channel has been initialized */
    int FlushonFsync;   /* VDMA Transactions are flushed & channel states
               reset on Frame Sync */
    int HasSG;          /* Whether hardware has SG engine */
    int IsRead;         /* Read or write channel */
    int HasDRE;         /* Whether support unaligned transfer */
    int LineBufDepth;   /* Depth of Channel Line Buffer FIFO */
    int LineBufThreshold;   /* Threshold point at which Channel Line
                 *  almost empty flag asserts high */
    int WordLength;     /* Word length */
    int NumFrames;  /* Number of frames to work on */

    UINTPTR HeadBdPhysAddr; /* Physical address of the first BD */
    UINTPTR HeadBdAddr;     /* Virtual address of the first BD */
    UINTPTR TailBdPhysAddr; /* Physical address of the last BD */
    UINTPTR TailBdAddr;     /* Virtual address of the last BD */
    int Hsize;          /* Horizontal size */
    int Vsize;          /* Vertical size saved for no-sg mode hw start */

    int AllCnt;         /* Total number of BDs */

    int GenLock;    /* Mm2s Gen Lock Mode */
    int S2MmSOF;    /* S2MM Start of Flag */
    int StreamWidth;     /* Stream Width */
    XAxiVdma_Bd BDs[XAXIVDMA_MAX_FRAMESTORE] __attribute__((__aligned__(32)));
                        /*Statically allocated BDs */
    u32 DbgFeatureFlags; /* Debug Parameter Flags */
    int AddrWidth;
    int direction;  /* Determines whether Read or write channel */
}XAxiVdma_Channel;

/* Duplicate layout of XAxiVdma_DmaSetup
 *
 * So to remove the dependency on xaxivdma.h
 */
typedef struct {
    int VertSizeInput;      /**< Vertical size input */
    int HoriSizeInput;      /**< Horizontal size input */
    int Stride;             /**< Stride */
    int FrameDelay;         /**< Frame Delay */

    int EnableCircularBuf;  /**< Circular Buffer Mode? */
    int EnableSync;         /**< Gen-Lock Mode? */
    int PointNum;           /**< Master we synchronize with */
    int EnableFrameCounter; /**< Frame Counter Enable */
    UINTPTR FrameStoreStartAddr[XAXIVDMA_MAX_FRAMESTORE];
                            /**< Start Addresses of Frame Store Buffers. */
    int FixedFrameStoreAddr;/**< Fixed Frame Store Address index */
    int GenLockRepeat;      /**< Gen-Lock Repeat? */
}XAxiVdma_ChannelSetup;


/**
 * VDMA data transfer direction
 */
#define XAXIVDMA_WRITE       1        /**< DMA transfer into memory */
#define XAXIVDMA_READ        2        /**< DMA transfer from memory */

/**
 * Frame Sync Source Selection
 */
#define XAXIVDMA_CHAN_FSYNC     0
#define XAXIVDMA_CHAN_OTHER_FSYNC   1
#define XAXIVDMA_S2MM_TUSER_FSYNC   2

/**
 * GenLock Source Selection
 */
#define XAXIVDMA_EXTERNAL_GENLOCK   0
#define XAXIVDMA_INTERNAL_GENLOCK   1

/**
 * GenLock Mode Constants
 */
#define XAXIVDMA_GENLOCK_MASTER     0
#define XAXIVDMA_GENLOCK_SLAVE      1
#define XAXIVDMA_DYN_GENLOCK_MASTER 2
#define XAXIVDMA_DYN_GENLOCK_SLAVE  3

/**
 * Interrupt type for setting up callback
 */
#define XAXIVDMA_HANDLER_GENERAL   1  /**< Non-Error Interrupt Type */
#define XAXIVDMA_HANDLER_ERROR     2  /**< Error Interrupt Type */

/**
 * Flag to signal that device is ready to be used
 */
#define XAXIVDMA_DEVICE_READY      0x11111111

/**
 * Debug Configuration Parameter Constants (C_ENABLE_DEBUG_INFO_*)
 */
#define XAXIVDMA_ENABLE_DBG_THRESHOLD_REG   0x01
#define XAXIVDMA_ENABLE_DBG_FRMSTORE_REG    0x02
#define XAXIVDMA_ENABLE_DBG_FRM_CNTR    0x04
#define XAXIVDMA_ENABLE_DBG_DLY_CNTR    0x08
#define XAXIVDMA_ENABLE_DBG_ALL_FEATURES    0x0F

/* Defined for backward compatiblity.
 * This  is a typical DMA Internal Error, which on detection doesnt require a
 * reset (as opposed to other errors). So user on seeing this need only to
 * reinitialize channels.
 *
 */
#ifndef XST_VDMA_MISMATCH_ERROR
#define XST_VDMA_MISMATCH_ERROR 1430
#endif


/**
 * Debug Configuration Parameter Constants (C_ENABLE_DEBUG_INFO_*)
 */
#define XAXIVDMA_ENABLE_DBG_THRESHOLD_REG   0x01
#define XAXIVDMA_ENABLE_DBG_FRMSTORE_REG    0x02
#define XAXIVDMA_ENABLE_DBG_FRM_CNTR    0x04
#define XAXIVDMA_ENABLE_DBG_DLY_CNTR    0x08

/*****************************************************************************/
/**
 * Callback type for general interrupts
 *
 * @param   CallBackRef is a callback reference passed in by the upper layer
 *          when setting the callback functions, and passed back to the
 *          upper layer when the callback is called.
 * @param   InterruptTypes indicates the detailed type(s) of the interrupt.
 *          Its value equals 'OR'ing one or more XAXIVDMA_IXR_* values defined
 *          in xaxivdma_hw.h
 *****************************************************************************/
typedef void (*XAxiVdma_CallBack) (void *CallBackRef, u32 InterruptTypes);

/*****************************************************************************/
/**
 * Callback type for Error interrupt.
 *
 * @param   CallBackRef is a callback reference passed in by the upper layer
 *          when setting the callback function, and it is passed back to the
 *          upper layer when the callback is called.
 * @param   ErrorMask is a bit mask indicating the cause of the error. Its
 *          value equals 'OR'ing one or more XAXIVDMA_IXR_* values defined in
 *          xaxivdma_hw.h
 *****************************************************************************/
typedef void (*XAxiVdma_ErrorCallBack) (void *CallBackRef, u32 ErrorMask);


/**
 * Channel callback functions
 */
typedef struct {
    XAxiVdma_CallBack CompletionCallBack; /**< Call back for completion intr */
    void *CompletionRef;                  /**< Call back ref */

    XAxiVdma_ErrorCallBack ErrCallBack;   /**< Call back for error intr */
    void *ErrRef;                         /**< Call back ref */
} XAxiVdma_ChannelCallBack;

/**
 * The XAxiVdma driver instance data.
 */
typedef struct {
    UINTPTR BaseAddr;                   /**< Memory address for this device */
    int HasSG;                      /**< Whether hardware has SG engine */
    int IsReady;                    /**< Whether driver is initialized */

    int MaxNumFrames;                /**< Number of frames to work on */
    int HasMm2S;                    /**< Whether hw build has read channel */
    int HasMm2SDRE;                 /**< Whether read channel has DRE */
    int HasS2Mm;                    /**< Whether hw build has write channel */
    int HasS2MmDRE;                 /**< Whether write channel has DRE */
    int EnableVIDParamRead;     /**< Read Enable for video parameters in
                      *  direct register mode */
    int UseFsync;               /**< DMA operations synchronized to
                      * Frame Sync */
    int InternalGenLock;        /**< Internal Gen Lock */
    XAxiVdma_ChannelCallBack ReadCallBack;  /**< Call back for read channel */
    XAxiVdma_ChannelCallBack WriteCallBack; /**< Call back for write channel */

    XAxiVdma_Channel ReadChannel;  /**< Channel to read from memory */
    XAxiVdma_Channel WriteChannel; /**< Channel to write to memory */
    int AddrWidth;        /**< Address Width */
} XAxiVdma;

/**
 * The XAxiVdma_DmaSetup structure contains all the necessary information to
 * start a frame write or read.
 *
 */
typedef struct {
    int VertSizeInput;      /**< Vertical size input */
    int HoriSizeInput;      /**< Horizontal size input */
    int Stride;             /**< Stride */
    int FrameDelay;         /**< Frame Delay */

    int EnableCircularBuf;  /**< Circular Buffer Mode? */
    int EnableSync;         /**< Gen-Lock Mode? */
    int PointNum;           /**< Master we synchronize with */
    int EnableFrameCounter; /**< Frame Counter Enable */
    UINTPTR FrameStoreStartAddr[XAXIVDMA_MAX_FRAMESTORE];
                            /**< Start Addresses of Frame Store Buffers. */
    int FixedFrameStoreAddr;/**< Fixed Frame Store Address index */
    int GenLockRepeat;      /**< Gen-Lock Repeat? */
} XAxiVdma_DmaSetup;

typedef struct {
    u16 DeviceId;         /**< DeviceId is the unique ID  of the device */
    UINTPTR BaseAddress;      /**< BaseAddress is the physical base address of the
                            *  device's registers */
    u16 MaxFrameStoreNum; /**< The maximum number of Frame Stores */
    int HasMm2S;          /**< Whether hw build has read channel */
    int HasMm2SDRE;       /**< Read channel supports unaligned transfer */
    int Mm2SWordLen;      /**< Read channel word length */
    int HasS2Mm;          /**< Whether hw build has write channel */
    int HasS2MmDRE;       /**< Write channel supports unaligned transfer */
    int S2MmWordLen;      /**< Write channel word length */
    int HasSG;            /**< Whether hardware has SG engine */
    int EnableVIDParamRead;
              /**< Read Enable for video parameters in direct
                *  register mode */
    int UseFsync;     /**< DMA operations synchronized to Frame Sync */
    int FlushonFsync;     /**< VDMA Transactions are flushed & channel states
                *   reset on Frame Sync */
    int Mm2SBufDepth;     /**< Depth of Read Channel Line Buffer FIFO */
    int S2MmBufDepth;     /**< Depth of Write Channel Line Buffer FIFO */
    int Mm2SGenLock;      /**< Mm2s Gen Lock Mode */
    int S2MmGenLock;      /**< S2Mm Gen Lock Mode */
    int InternalGenLock;  /**< Internal Gen Lock */
    int S2MmSOF;      /**< S2MM Start of Flag Enable */
    int Mm2SStreamWidth;  /**< MM2S TData Width */
    int S2MmStreamWidth;  /**< S2MM TData Width */
    int Mm2SThresRegEn;   /**< MM2S Threshold Register Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_1
                               configuration parameter */
    int Mm2SFrmStoreRegEn;/**< MM2S Frame Store Register Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_5
                               configuration parameter */
    int Mm2SDlyCntrEn;    /**< MM2S Delay Counter (Control Reg) Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_6
                               configuration parameter */
    int Mm2SFrmCntrEn;    /**< MM2S Frame Counter (Control Reg) Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_7
                               configuration parameter */
    int S2MmThresRegEn;   /**< S2MM Threshold Register Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_9
                               configuration parameter */
    int S2MmFrmStoreRegEn;/**< S2MM Frame Store Register Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_13
                               configuration parameter */
    int S2MmDlyCntrEn;    /**< S2MM Delay Counter (Control Reg) Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_14
                               configuration parameter */
    int S2MmFrmCntrEn;    /**< S2MM Frame Counter (Control Reg) Enable Flag
                               This corresponds to C_ENABLE_DEBUG_INFO_15
                               configuration parameter */
    int EnableAllDbgFeatures;/**< Enable all Debug features
                               This corresponds to C_ENABLE_DEBUG_ALL
                               configuration parameter */
    int AddrWidth;        /**< Address Width */
} XAxiVdma_Config;




#endif /* __XILTYPES_H__ */
