#ifndef _NOVATEK_NVT_TS_H_
#define _NOVATEK_NVT_TS_H_

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <PiDxe.h>
#include <Protocol/AbsolutePointer.h>
#include <Protocol/EFII2C.h>
#include <Protocol/EFITlmm.h>

#define NVT_TS_TOUCH_START 0x00
#define NVT_TS_TOUCH_SIZE 6

// I2C Addresses
#define I2C_HW_Address 0x62
#define I2C_FW_Address 0x01
#define I2C_BLDR_Address 0x01

// Event Map Offsets
#define EVENT_MAP_HOST_CMD 0x50
#define EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE 0x51
#define EVENT_MAP_RESET_COMPLETE 0x60
#define EVENT_MAP_FWINFO 0x78
#define EVENT_MAP_PROJECTID 0x9A

// Mode Commands
#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB
#define FREQ_HOP_DISABLE 0x66
#define FREQ_HOP_ENABLE 0x65

// ==================== NEW: Memory Map Structure ====================
typedef struct _NVT_TS_MEM_MAP {
  UINT32 EVENT_BUF_ADDR;
  UINT32 RAW_PIPE0_ADDR;
  UINT32 RAW_PIPE1_ADDR;
  UINT32 BASELINE_ADDR;
  UINT32 BASELINE_BTN_ADDR;
  UINT32 DIFF_PIPE0_ADDR;
  UINT32 DIFF_PIPE1_ADDR;
  UINT32 RAW_BTN_PIPE0_ADDR;
  UINT32 RAW_BTN_PIPE1_ADDR;
  UINT32 DIFF_BTN_PIPE0_ADDR;
  UINT32 DIFF_BTN_PIPE1_ADDR;
  UINT32 READ_FLASH_CHECKSUM_ADDR;
  UINT32 RW_FLASH_DATA_ADDR;
} NVT_TS_MEM_MAP;

// NT36672A Memory Map (from Linux driver)
static const NVT_TS_MEM_MAP NT36672A_MemoryMap = {
    .EVENT_BUF_ADDR           = 0x21C00,
    .RAW_PIPE0_ADDR           = 0x20000,
    .RAW_PIPE1_ADDR           = 0x23000,
    .BASELINE_ADDR            = 0x20BFC,
    .BASELINE_BTN_ADDR        = 0x23BFC,
    .DIFF_PIPE0_ADDR          = 0x206DC,
    .DIFF_PIPE1_ADDR          = 0x236DC,
    .RAW_BTN_PIPE0_ADDR       = 0x20510,
    .RAW_BTN_PIPE1_ADDR       = 0x23510,
    .DIFF_BTN_PIPE0_ADDR      = 0x20BF0,
    .DIFF_BTN_PIPE1_ADDR      = 0x23BF0,
    .READ_FLASH_CHECKSUM_ADDR = 0x24000,
    .RW_FLASH_DATA_ADDR       = 0x24002,
};

// ==================== Tuning Registers ====================
#define NVT_REG_MONITOR_MODE 0x7000
#define NVT_REG_THDIFF 0x7100
#define NVT_REG_SENSIVITY 0x7200
#define NVT_REG_EDGE_FILTER_LEVEL 0xBA00
#define NVT_REG_EDGE_FILTER_ORIENTATION 0xBC00

// Registers and Commands
#define NVT_TS_PARAMETERS_START 0x78

typedef enum {
  RESET_STATE_INIT = 0xA0, // IC reset
  RESET_STATE_REK,         // ReK baseline (0xA1)
  RESET_STATE_REK_FINISH,  // baseline is ready (0xA2)
  RESET_STATE_NORMAL_RUN,  // normal run (0xA3)
  RESET_STATE_MAX = 0xAF
} RST_COMPLETE_STATE;

/* These are offsets from NVT_TS_PARAMETERS_START */
#define NVT_TS_PARAMS_WIDTH 0x04
#define NVT_TS_PARAMS_HEIGHT 0x06
#define NVT_TS_PARAMS_MAX_TOUCH 0x09
#define NVT_TS_PARAMS_MAX_BUTTONS 0x0a
#define NVT_TS_PARAMS_IRQ_TYPE 0x0b
#define NVT_TS_PARAMS_WAKE_TYPE 0x0c
#define NVT_TS_PARAMS_CHIP_ID 0x0e
#define NVT_TS_PARAMS_SIZE 0x0f

#define NVT_TS_MAX_TOUCHES 10
#define NVT_TS_MAX_SIZE 4096
#define NVT_TS_TOUCH_INVALID 0xff
#define NVT_TS_TOUCH_SLOT_SHIFT 3
#define NVT_TS_TOUCH_TYPE_MASK 0x07
#define NVT_TS_TOUCH_NEW 0x01
#define NVT_TS_TOUCH_UPDATE 0x02
#define NVT_TS_TOUCH_RELEASE 0x00

#define TOUCH_FORCE_NUM 1000

#define TOUCH_DELAY_TO_COMMUNICATE 50000
#define TOUCH_POWER_RAIL_STABLE_TIME 10000

// 2.5ms = 400Hz polling (25,000 * 100ns = 2.5ms) - Fast Mode
#define TIMER_INTERVAL_TOUCH_POLL 20000

typedef struct _TOUCH_DATA {
  UINT16 TouchX;
  UINT16 TouchY;
  UINT16 TouchPressure;
  UINT16 TouchWidth;
  UINT8  TouchStatus;
} TOUCH_DATA, *PTOUCH_DATA;

// Full Touch Data Structure
#define POINT_DATA_LEN 65
typedef struct _NVT_TOUCH_DATA {
  UINT8 PointData[POINT_DATA_LEN + 1];
} NVT_TOUCH_DATA;

// FW Info Structure
typedef struct _NVT_ID_INFO {
  UINT8  FwVer;
  UINT8  XNum;
  UINT8  YNum;
  UINT16 AbsXMax;
  UINT16 AbsYMax;
  UINT8  MaxButtonNum;
  UINT16 Pid;
} NVT_ID_INFO;

// ==================== Tuning Configuration ====================
typedef struct _NVT_TUNING_CONFIG {
  UINT8   Sensitivity;     // 0-255, default 100
  UINT8   EdgeFilterLevel; // 0-255, default 15
  UINT8   EdgeOrientation; // 0=portrait, 1=landscape
  UINT8   ThresholdDiff;   // Touch threshold differential
  BOOLEAN FreqHopEnabled;  // Frequency hopping
} NVT_TUNING_CONFIG;

// Novatek device
extern EFI_GUID gNovatekTouchDeviceProtocolGuid;

typedef struct _NOVATEK_I2C_DEVICE {
  UINT32                  Signature;
  UINT32                  XMax;
  UINT32                  YMax;
  UINT32                  XMin;
  UINT32                  YMin;
  BOOLEAN                 XInverted;
  BOOLEAN                 YInverted;
  UINT32                  ControllerResetPin;
  UINT32                  ControllerInterruptPin;
  UINT32                  ControllerVddPin;
  UINT32                  ControllerVddIoPin;
  BOOLEAN                 ControllerVddCtlActiveLow;
  UINT32                  ControllerI2cDevice;
  EFI_QCOM_TLMM_PROTOCOL *GpioTlmmProtocol;
  EFI_QCOM_I2C_PROTOCOL  *I2cQupProtocol;
  I2C_SLAVE_CONFIG        SlaveCfg;
} NOVATEK_I2C_DEVICE;

#define NVT_DEV_INSTANCE_SIGNATURE SIGNATURE_32('n', 'v', 't', 'd')

// Novatek driver internals
typedef struct _NVT_INTERNAL_DATA {
  UINT32 Signature;

  // Ghost touch debouncing
  UINT16 LastRawX;
  UINT16 LastRawY;
  UINT32 StationaryTouchCount;
  UINT16 LastX;
  UINT16 LastY;

  EFI_ABSOLUTE_POINTER_PROTOCOL AbsPointerProtocol;
  EFI_ABSOLUTE_POINTER_MODE     AbsPointerMode;
  EFI_EVENT                     PollingTimerEvent;
  EFI_EVENT                     TouchWorkerEvent;
  BOOLEAN                       Initialized;

  UINT32 TouchDataAddress;
  UINT32 ChecksumAddress;
  UINT32 FlashDataAddress;

  VOID                     *I2cController;
  BOOLEAN                   StateChanged;
  NOVATEK_I2C_DEVICE       *NvtDevice;
  EFI_UNICODE_STRING_TABLE *ControllerNameTable;
  UINT8                    *Buf;
  UINT8                     LastPointData[8];

  BOOLEAN IsBusy;
  UINT32  CurrentSlaveAddress;

  NVT_ID_INFO IdInfo;
  UINT32      EventBufAddr;
  INT32       LastPacketId;
  BOOLEAN     IsTouched;
  UINT32      IrqHighCount;
  BOOLEAN     FirmwareBroken;

  // ==================== NEW: Memory Map & Tuning ====================
  const NVT_TS_MEM_MAP *MemMap;
  NVT_TUNING_CONFIG     TuningConfig;
  UINT8                 CurrentMode;   // NORMAL_MODE, TEST_MODE_1, TEST_MODE_2
  UINT8                 CarrierSystem; // 0 = normal, 1 = RSS

  // GPIO Cache (Optimization)
  INT32 LastSdaLevel; // -1=Unknown, 0=Low, 1=High
  INT32 LastSclLevel; // -1=Unknown, 0=Low, 1=High
} NVT_INTERNAL_DATA;

#define NVT_TCH_INSTANCE_SIGNATURE SIGNATURE_32('n', 'v', 't', 's')
#define NVT_TCH_INSTANCE_FROM_ABSTCH_THIS(a)                                   \
  CR(a, NVT_INTERNAL_DATA, AbsPointerProtocol, NVT_TCH_INSTANCE_SIGNATURE)

// ==================== Function Declarations ====================

// Core I2C functions
EFI_STATUS EFIAPI NvtI2cRead(
    IN NVT_INTERNAL_DATA *Instance, IN UINT16 Address, OUT UINT8 *Data,
    IN UINT32 Length);

EFI_STATUS EFIAPI NvtI2cWrite(
    IN NVT_INTERNAL_DATA *Instance, IN UINT16 Address, IN UINT8 *Data,
    IN UINT32 Length);

EFI_STATUS EFIAPI NvtI2cWriteRaw(
    IN NVT_INTERNAL_DATA *Instance, IN UINT8 *Data, IN UINT32 Length);

EFI_STATUS EFIAPI NvtI2cReceive(
    IN NVT_INTERNAL_DATA *Instance, OUT UINT8 *Data, IN UINT32 Length);
// ==================== NEW: Advanced Functions ====================

// Set XDATA page for high memory access
EFI_STATUS NvtSetXdataPage(IN NVT_INTERNAL_DATA *Instance, IN UINT32 Address);

// Change operating mode
EFI_STATUS NvtChangeMode(IN NVT_INTERNAL_DATA *Instance, IN UINT8 Mode);

// Get current FW pipe (0 or 1)
UINT8 NvtGetFwPipe(IN NVT_INTERNAL_DATA *Instance);

// Tuning functions
EFI_STATUS NvtSetSensitivity(IN NVT_INTERNAL_DATA *Instance, IN UINT8 Level);

EFI_STATUS NvtConfigureEdgeFilter(
    IN NVT_INTERNAL_DATA *Instance, IN UINT8 Level, IN UINT8 Orientation);

EFI_STATUS
NvtSetThresholdDiff(IN NVT_INTERNAL_DATA *Instance, IN UINT8 Threshold);

EFI_STATUS NvtSetFreqHop(IN NVT_INTERNAL_DATA *Instance, IN BOOLEAN Enable);

EFI_STATUS NvtApplyTuningConfig(IN NVT_INTERNAL_DATA *Instance);

// Existing functions
VOID       NvtSwReset(IN NVT_INTERNAL_DATA *Instance);
EFI_STATUS NvtBootloaderReset(NVT_INTERNAL_DATA *Instance);
EFI_STATUS EFIAPI
NvtCheckFwResetState(IN NVT_INTERNAL_DATA *Instance, IN UINT8 ExpectedState);
EFI_STATUS NvtClearFwStatus(IN NVT_INTERNAL_DATA *Instance);
EFI_STATUS NvtCheckFwStatus(IN NVT_INTERNAL_DATA *Instance);
EFI_STATUS NvtReadPid(NVT_INTERNAL_DATA *Instance);
EFI_STATUS NvtGetFwInfo(NVT_INTERNAL_DATA *Instance);
EFI_STATUS NvtCheckChipVersion(IN NVT_INTERNAL_DATA *Instance);
EFI_STATUS EFIAPI
NvtSetSlaveAddress(NVT_INTERNAL_DATA *Instance, UINT32 Address);

EFI_STATUS AbsPReset(
    IN EFI_ABSOLUTE_POINTER_PROTOCOL *This, IN BOOLEAN ExtendedVerification);

EFI_STATUS AbsPGetState(
    IN EFI_ABSOLUTE_POINTER_PROTOCOL  *This,
    IN OUT EFI_ABSOLUTE_POINTER_STATE *State);

EFI_STATUS        AbsStartPolling(IN NVT_INTERNAL_DATA *Instance);
VOID EFIAPI       AbsPWaitForInput(IN EFI_EVENT Event, IN VOID *Context);
VOID EFIAPI       SyncPollCallback(IN EFI_EVENT Event, IN VOID *Context);
EFI_STATUS EFIAPI NvtPowerUpController(NVT_INTERNAL_DATA *Instance);
EFI_STATUS EFIAPI
NvtGetTouchData(NVT_INTERNAL_DATA *Instance, IN PTOUCH_DATA DataBuffer);

// Driver binding
extern EFI_DRIVER_BINDING_PROTOCOL  gNvtDriverBinding;
extern EFI_COMPONENT_NAME_PROTOCOL  gNvtDriverComponentName;
extern EFI_COMPONENT_NAME2_PROTOCOL gNvtDriverComponentName2;

EFI_STATUS EFIAPI NvtAbsolutePointerDriverBindingSupported(
    IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
    IN EFI_DEVICE_PATH_PROTOCOL *RemainingDevicePath);

EFI_STATUS EFIAPI NvtAbsolutePointerDriverBindingStart(
    IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
    IN EFI_DEVICE_PATH_PROTOCOL *RemainingDevicePath);

EFI_STATUS EFIAPI NvtAbsolutePointerDriverBindingStop(
    IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
    IN UINTN NumberOfChildren, IN EFI_HANDLE *ChildHandleBuffer);

#endif
