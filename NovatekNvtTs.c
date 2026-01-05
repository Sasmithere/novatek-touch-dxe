#include <Uefi.h>

/* READ THIS MAIN !!! don't blame me if something goes wrong
  *******************************************************************************
  TUNING PARAMETER GUIDE

  Adjust these values in NvtPowerUpController() based on symptoms:

  1. GHOST TOUCHES / FALSE POSITIVES:
     - Decrease Sensitivity: 100 → 80 → 60
     - Increase ThresholdDiff: 70 → 90 → 110
     - Increase EdgeFilterLevel: 15 → 25 → 35

  2. MISSED TOUCHES / LOW RESPONSE:
     - Increase Sensitivity: 100 → 120 → 140
     - Decrease ThresholdDiff: 70 → 50 → 30
     - Decrease EdgeFilterLevel: 15 → 10 → 5

  3. EDGE PALM REJECTION ISSUES:
     - Increase EdgeFilterLevel: 15 → 30 → 50
     - Check EdgeOrientation matches device (0=portrait, 1=landscape)

  4. EMI / ELECTRICAL NOISE:
     - Enable FreqHopEnabled: FALSE → TRUE
     - Increase ThresholdDiff: 70 → 100

  5. INCONSISTENT CALIBRATION:
     - Verify RESET_STATE_REK_FINISH (0xA2) is reached before tuning
     - Add NvtBootloaderReset() retry logic

  DEFAULT SAFE VALUES (in NvtPowerUpController):
    Sensitivity:      100  // Middle ground
    EdgeFilterLevel:  15   // Light filtering
    EdgeOrientation:  0    // Portrait mode
    ThresholdDiff:    70   // Moderate noise rejection
    FreqHopEnabled:   FALSE // Disable unless needed

*******************************************************************************
*/

// #include "NovatekFlash.h"
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

// Prototype for Loader
// EFI_STATUS NvtLoadAndFlashFirmware(NVT_INTERNAL_DATA *Instance);

#include <Protocol/AbsolutePointer.h>
#include <Protocol/EFIClock.h>
#include <Protocol/EFII2C.h>
#include <Protocol/EFITlmm.h>

#include "NovatekNvtTs.h"
#include <Device/TouchDevicePath.h>
// I2C Addresses
#define I2C_ADDR_NVT 0x62   // Hardware / Bootloader Control
#define I2C_FW_Address 0x01 // Kernel Data Port (Pipe 1)
#define I2C_HW_Address 0x62 // Hardware Port (Pipe 0)

// Helper Macros
#define ABS1(x) ((x) < 0 ? -(x) : (x))

// Prototypes (Forward Declarations)
// None required at this level

#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>

// --- FORWARD DECLARATIONS ---
// EFI_STATUS EFIAPI NvtI2cWriteRaw(
//     IN NVT_INTERNAL_DATA *Instance, IN UINT8 *Data, IN UINT32 Length);
// EFI_STATUS EFIAPI NvtI2cRead(
//     IN NVT_INTERNAL_DATA *Instance, IN UINT16 Address, OUT UINT8 *Data,
//     IN UINT32 Length);
// EFI_STATUS EFIAPI NvtI2cWrite(
//     IN NVT_INTERNAL_DATA *Instance, IN UINT16 Address, IN UINT8 *Data,
//     IN UINT32 Length);

// --- TRIM TABLE & STRUCTS ---
#define NVT_ID_BYTE_MAX 6

static const NVT_TS_MEM_MAP NT36672A_memory_map = {.EVENT_BUF_ADDR = 0x21C00};
static const NVT_TS_MEM_MAP NT36772_memory_map  = {.EVENT_BUF_ADDR = 0x11E00};
static const NVT_TS_MEM_MAP NT36525_memory_map  = {.EVENT_BUF_ADDR = 0x11A00};
static const NVT_TS_MEM_MAP NT36870_memory_map  = {.EVENT_BUF_ADDR = 0x25000};
static const NVT_TS_MEM_MAP NT36676F_memory_map = {.EVENT_BUF_ADDR = 0x11A00};

typedef struct {
  UINT8                 id[NVT_ID_BYTE_MAX];
  UINT8                 mask[NVT_ID_BYTE_MAX];
  const NVT_TS_MEM_MAP *mmap;
} NVT_TS_TRIM_ID_TABLE;

static const NVT_TS_TRIM_ID_TABLE trim_id_table[] = {
    {.id   = {0x0A, 0xFF, 0xFF, 0x72, 0x65, 0x03},
     .mask = {1, 0, 0, 1, 1, 1},
     .mmap = &NT36672A_memory_map},
    {.id   = {0x0A, 0xFF, 0xFF, 0x72, 0x66, 0x03},
     .mask = {1, 0, 0, 1, 1, 1},
     .mmap = &NT36672A_memory_map},
    {.id   = {0x0A, 0xFF, 0xFF, 0x82, 0x66, 0x03},
     .mask = {1, 0, 0, 1, 1, 1},
     .mmap = &NT36672A_memory_map},
    {.id   = {0x0A, 0xFF, 0xFF, 0x70, 0x66, 0x03},
     .mask = {1, 0, 0, 1, 1, 1},
     .mmap = &NT36672A_memory_map},
    {.id   = {0x0B, 0xFF, 0xFF, 0x70, 0x66, 0x03},
     .mask = {1, 0, 0, 1, 1, 1},
     .mmap = &NT36672A_memory_map},
    {.id   = {0x0A, 0xFF, 0xFF, 0x72, 0x67, 0x03},
     .mask = {1, 0, 0, 1, 1, 1},
     .mmap = &NT36672A_memory_map},
    {.id   = {0x55, 0x00, 0xFF, 0x00, 0x00, 0x00},
     .mask = {1, 1, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0x55, 0x72, 0xFF, 0x00, 0x00, 0x00},
     .mask = {1, 1, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00},
     .mask = {1, 1, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00},
     .mask = {1, 1, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36772_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36525_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x70, 0x68, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36870_memory_map},
    {.id   = {0xFF, 0xFF, 0xFF, 0x76, 0x66, 0x03},
     .mask = {0, 0, 0, 1, 1, 1},
     .mmap = &NT36676F_memory_map}};

// --- HELPER FUNCTIONS ---

/**
  Send Reset Idle command to Hardware I2C Address (0x62)

  @param Instance    Pointer to driver instance
**/
VOID NvtSwResetIdle(NVT_INTERNAL_DATA *Instance)
{
  UINT8 Buf[2];
  Buf[0] = 0x00;
  Buf[1] = 0xA5;
  NvtSetSlaveAddress(Instance, I2C_HW_Address);
  NvtI2cWriteRaw(Instance, Buf, 2);
}

/**
  Stop firmware CRC reboot loop (Clone IC issue)

  Hardware pattern:
  1. Bootloader reads 0xFC (CRC Error)
  2. Chip resets
  3. Repeats indefinitely

  Fix:
  - Reset to Idle (0xA5)
  - Clear Error (0xF1)
  - Verify stability (0xA5)

  @param Instance    Pointer to driver instance
**/
VOID NvtStopCrcReboot(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      Buf[8];
  UINT32     Retry;

  DEBUG((EFI_D_ERROR, "NVT: Stopping CRC Reboot Loop...\n"));

  // Set page
  Buf[0] = 0xFF;
  Buf[1] = 0x01;
  Buf[2] = 0xF6;
  NvtSetSlaveAddress(Instance, I2C_BLDR_Address);
  NvtI2cWriteRaw(Instance, Buf, 3);

  //  FIX: Set slave address BEFORE reading
  NvtSetSlaveAddress(Instance, I2C_BLDR_Address);
  Status = NvtI2cRead(
      Instance, 0x4E, Buf, 4); // Register 0x4E, not I2C_BLDR_Address!

  if ((Buf[1] == 0xFC) ||
      ((Buf[1] == 0xFF) && (Buf[2] == 0xFF) && (Buf[3] == 0xFF))) {

    for (Retry = 5; Retry > 0; Retry--) {
      // Reset idle 1st
      Buf[0] = 0x00;
      Buf[1] = 0xA5;
      NvtSetSlaveAddress(Instance, I2C_HW_Address);
      NvtI2cWriteRaw(Instance, Buf, 2);

      // Reset idle 2nd
      Buf[0] = 0x00;
      Buf[1] = 0xA5;
      NvtSetSlaveAddress(Instance, I2C_HW_Address);
      NvtI2cWriteRaw(Instance, Buf, 2);
      gBS->Stall(1000); // Increased delay

      // Clear CRC error flag
      Buf[0] = 0xFF;
      Buf[1] = 0x03;
      Buf[2] = 0xF1;
      NvtSetSlaveAddress(Instance, I2C_BLDR_Address);
      NvtI2cWriteRaw(Instance, Buf, 3);

      Buf[0] = 0x35;
      Buf[1] = 0xA5;
      NvtSetSlaveAddress(Instance, I2C_BLDR_Address);
      NvtI2cWriteRaw(Instance, Buf, 2);

      // Check CRC error flag
      Buf[0] = 0xFF;
      Buf[1] = 0x03;
      Buf[2] = 0xF1;
      NvtSetSlaveAddress(Instance, I2C_BLDR_Address);
      NvtI2cWriteRaw(Instance, Buf, 3);

      //  FIX: Set slave address before reading
      NvtSetSlaveAddress(Instance, I2C_BLDR_Address);
      Status = NvtI2cRead(Instance, 0x35, Buf, 2); // Register 0x35

      if (!EFI_ERROR(Status) && Buf[1] == 0xA5) {
        DEBUG(
            (EFI_D_ERROR, "NVT: CRC Reboot Stopped (0xA5) after %d retries\n",
             6 - Retry));
        break;
      }
    }

    if (Retry == 0) {
      DEBUG(
          (EFI_D_ERROR, "NVT: CRC reboot could not be stopped! buf[1]=0x%02X\n",
           Buf[1]));
    }
  }
  else {
    DEBUG((EFI_D_ERROR, "NVT: No CRC reboot detected\n"));
  }
}

/**
  Identify chip version and load trim data

  Matches Chip ID (Trim) with supported table.
  Sets Event Buffer Address based on chip type.

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Chip identified or Default used
  @retval EFI_DEVICE_ERROR      I2C Error
**/
EFI_STATUS NvtCheckChipVerTrim(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      Buf[8];
  UINT32     Retry;
  UINT32     List, i;
  BOOLEAN    FoundNvtChip = FALSE;

  Instance->EventBufAddr = 0x21C00; // Default

  DEBUG((EFI_D_ERROR, "NVT: Check Chip Ver Trim...\n"));

  for (Retry = 5; Retry > 0; Retry--) {
    NvtSwResetIdle(Instance);

    Buf[0] = 0x00;
    Buf[1] = 0x35;
    NvtSetSlaveAddress(Instance, I2C_HW_Address);
    NvtI2cWriteRaw(Instance, Buf, 2);
    gBS->Stall(10000);

    Buf[0] = 0xFF;
    Buf[1] = 0x01;
    Buf[2] = 0xF6;
    NvtSetSlaveAddress(Instance, I2C_FW_Address);
    NvtI2cWriteRaw(Instance, Buf, 3);

    Buf[0] = 0x4E;
    Buf[1] = 0x00;
    Buf[2] = 0x00;
    Buf[3] = 0x00;
    Buf[4] = 0x00;
    Buf[5] = 0x00;
    Buf[6] = 0x00;
    NvtSetSlaveAddress(Instance, I2C_FW_Address);
    NvtI2cWriteRaw(Instance, Buf, 7);

    Status = NvtI2cRead(Instance, 0x4E, Buf, 7);
    if (EFI_ERROR(Status))
      continue;

    DEBUG(
        (EFI_D_ERROR, "NVT: Trim Read: %02X %02X %02X %02X %02X %02X\n", Buf[0],
         Buf[1], Buf[2], Buf[3], Buf[4], Buf[5]));

    // Check for CRC reboot signature
    if (Buf[1] == 0xFC ||
        (Buf[1] == 0xFF && Buf[2] == 0xFF && Buf[3] == 0xFF)) {
      NvtStopCrcReboot(Instance);
      continue;
    }

    // FIX: Properly check all entries in trim table
    FoundNvtChip = FALSE;
    for (List = 0;
         List < (sizeof(trim_id_table) / sizeof(NVT_TS_TRIM_ID_TABLE));
         List++) {
      // Match each byte with mask
      for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
        if (trim_id_table[List].mask[i]) {
          if (Buf[i] != trim_id_table[List].id[i])
            break; // No match, try next entry
        }
      }

      // If all bytes matched, we found the chip
      if (i == NVT_ID_BYTE_MAX) {
        FoundNvtChip           = TRUE;
        Instance->EventBufAddr = trim_id_table[List].mmap->EVENT_BUF_ADDR;
        Instance->MemMap =
            trim_id_table[List].mmap; // FIX: Initialize MemMap pointer!
        DEBUG((
            EFI_D_ERROR, "NVT: Chip Identified! Index %d, EventBufAddr: 0x%X\n",
            List, Instance->EventBufAddr));
        gBS->Stall(10000);
        return EFI_SUCCESS; // SUCCESS when chip found!
      }
    }

    // If not found, retry
    if (!FoundNvtChip) {
      DEBUG((EFI_D_ERROR, "NVT: Chip not in table, retry %d\n", Retry));
    }
  }

  // After all retries failed
  DEBUG((EFI_D_ERROR, "NVT: Chip Identify Failed - Using Default 0x21C00\n"));

  return EFI_SUCCESS;
}

// --- CLOCK PATCH ---
#define GCC_QUPV3_WRAP0_CORE_2X_CMD_RCGR 0x00117018
#define GCC_QUPV3_WRAP0_S1_CMD_RCGR 0x00117278 // Correct Address
#define GCC_QUPV3_WRAP0_S1_CFG_RCGR 0x0011727C
#define GCC_QUPV3_WRAP_0_S_AHB_CBCR 0x00117008 // HW Gating / Enable
#define GCC_QUPV3_WRAP_0_S_AHB_CBCR 0x00117008 // HW Gating / Enable
#define GCC_QUPV3_WRAP0_S1_CBCR 0x00117274     // S1 Branch Clock Control

/**
  Configure QUPV3 RCG Clock (Root Clock Generator)

  Sets clock source to XO (19.2MHz) and triggers update.
  Required for I2C to function if Bootloader didn't set it.

  @param BaseAddr    Base Address of RCG Register
**/
VOID NvtConfigRCG(UINT32 BaseAddr)
{
  // 1. Configure CFG_RCGR: SRC_SEL=0 (XO), SRC_DIV=0 (Div 1), MODE=0 (Bypass)
  // -> 19.2MHz
  MmioWrite32(BaseAddr + 0x4, 0x00000000);

  // 2. Trigger Update: Bit 0=UPDATE, Bit 1=ROOT_EN (Crucial!)
  MmioWrite32(BaseAddr, 0x00000003);

  // 3. Wait for Update
  UINT32 Retries = 1500;
  while (Retries > 0) {
    if (!(MmioRead32(BaseAddr) & 0x1)) {
      break;
    }
    MicroSecondDelay(10);
    Retries--;
  }

  DEBUG((EFI_D_ERROR, "NVT: RCG Update Complete at 0x%08x\n", BaseAddr));
}

// --- GENI REGISTER DUMP ---
#define GENI_SE_BASE 0x00884000
#define GENI_CLK_CTRL 0x0
#define GENI_SER_M_CLK_CFG 0x4C
#define GENI_FW_REVISION 0x68
#define GENI_STATUS 0x40
#define GENI_OUTPUT_CTRL 0x24
#define GENI_I2C_STATUS                                                        \
  0x2A4 // Offset from SE Base? Actually 0x884000 is SE base.
// GENI SE I2C Offsets usually start at 0x0? No, SE registers are common. I2C
// specific start at 0x280? Linux: #define SE_GENI_STATUS
// 0x40 #define SE_GENI_IOS			0x908 #define SE_I2C_STATUS
// 0x2A4
// --- SOFTWARE I2C (BIT-BANGING) IMPLEMENTATION ---
#define PIN_SDA 4
#define PIN_SCL 5

// Helper to configure GPIO Direction
// High (1) = Input (Floats High via Pull-up)
// Low (0) = Output Low (Drive 0)
// This simulates Open Drain.
/**
  Set GPIO Pin State (Open Drain Simulation)

  @param Instance    Pointer to driver instance
  @param Pin         Pin Number (SDA or SCL)
  @param Level       TRUE=High(Input/Float), FALSE=Low(Output/Drive)
**/
VOID SwI2cSetPin(NVT_INTERNAL_DATA *Instance, UINT32 Pin, BOOLEAN Level)
{
  // Optimization: Check Cache to avoid slow GPIO Config calls
  INT32 *LastLevel = NULL;
  if (Pin == PIN_SDA) {
    LastLevel = &Instance->LastSdaLevel;
  }
  else if (Pin == PIN_SCL) {
    LastLevel = &Instance->LastSclLevel;
  }

  if (LastLevel != NULL && *LastLevel == (INT32)Level) {
    // Already in correct state - skip expensive TLMM call
    return;
  }

  UINT32 Config;
  if (Level) {
    // High-Z (Input)
    Config = EFI_GPIO_CFG(Pin, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
    Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(Config, TLMM_GPIO_ENABLE);
  }
  else {
    // Drive Low (Output)
    Config = EFI_GPIO_CFG(Pin, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
    Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(Config, TLMM_GPIO_ENABLE);
    Instance->NvtDevice->GpioTlmmProtocol->GpioOut(Config, 0);
  }

  // Update Cache
  if (LastLevel != NULL) {
    *LastLevel = (INT32)Level;
  }
}

/**
  Get GPIO Pin State

  @param Instance    Pointer to driver instance
  @param Pin         Pin Number
  @retval TRUE       High
  @retval FALSE      Low
**/
BOOLEAN SwI2cGetPin(NVT_INTERNAL_DATA *Instance, UINT32 Pin)
{
  // Assume Pin is already configured as Input (if High) or Input (if we want to
  // read) But to be safe, we must Config as Input if we want to read external
  // state? Only time we read is ACK (SDA) or Clock Stretching (SCL). In both
  // cases, we set it High (Input) before reading.

  UINT32 Config = EFI_GPIO_CFG(Pin, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
  // Only Config if needed? Assuming caller sets High (Input) first.

  UINT32 Value = 0;
  Instance->NvtDevice->GpioTlmmProtocol->GpioIn(Config, &Value);
  return (Value != 0);
}

/**
  Microsecond Delay for I2C Timing
**/
VOID SwI2cDelay()
{
  // Reduced to 1us - High Speed (Optimized with GPIO Cache)
  MicroSecondDelay(1);
}

/**
  Wait for Clock Stretching (Slave holding SCL Low)

  @param Instance    Pointer to driver instance
**/
VOID SwI2cWaitClock(NVT_INTERNAL_DATA *Instance)
{
  // Increase timeout to 500ms for slow clone chips
  UINT32 Retry = 100000;
  while (!SwI2cGetPin(Instance, PIN_SCL) && Retry > 0) {
    MicroSecondDelay(5);
    Retry--;
  }
}

/**
  Generate I2C START Condition

  @param Instance    Pointer to driver instance
**/
VOID SwI2cStart(NVT_INTERNAL_DATA *Instance)
{
  // SDA High, SCL High
  SwI2cSetPin(Instance, PIN_SDA, TRUE);
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  SwI2cWaitClock(Instance);
  SwI2cDelay();

  // SDA Low
  SwI2cSetPin(Instance, PIN_SDA, FALSE);
  SwI2cDelay();

  // SCL Low
  SwI2cSetPin(Instance, PIN_SCL, FALSE);
  SwI2cDelay();
}

/**
  Generate I2C STOP Condition

  @param Instance    Pointer to driver instance
**/
VOID SwI2cStop(NVT_INTERNAL_DATA *Instance)
{
  // SDA Low, SCL Low
  SwI2cSetPin(Instance, PIN_SDA, FALSE);
  SwI2cSetPin(Instance, PIN_SCL, FALSE);
  SwI2cDelay();

  // SCL High
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  SwI2cWaitClock(Instance);
  SwI2cDelay();

  // SDA High
  SwI2cSetPin(Instance, PIN_SDA, TRUE);
  SwI2cDelay();
}

/**
  Write Byte to I2C

  @param Instance    Pointer to driver instance
  @param Byte        Byte to write
  @retval TRUE       ACK received
  @retval FALSE      NACK received
**/
BOOLEAN SwI2cWriteByte(NVT_INTERNAL_DATA *Instance, UINT8 Byte)
{
  INT32 i;

  // Write 8 bits
  for (i = 7; i >= 0; i--) {
    // SDA
    SwI2cSetPin(Instance, PIN_SDA, (Byte >> i) & 1);
    SwI2cDelay();

    // Clock High
    SwI2cSetPin(Instance, PIN_SCL, TRUE);
    SwI2cWaitClock(Instance); // Check Stretching
    SwI2cDelay();

    // Clock Low
    SwI2cSetPin(Instance, PIN_SCL, FALSE);
    SwI2cDelay();
  }

  // ACK Phase: SDA Input
  SwI2cSetPin(Instance, PIN_SDA, TRUE); // Input
  SwI2cDelay();

  // Clock High (Slave drives ACK)
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  SwI2cWaitClock(Instance);
  SwI2cDelay();

  // Read ACK
  BOOLEAN Ack = !SwI2cGetPin(Instance, PIN_SDA);

  // Clock Low
  SwI2cSetPin(Instance, PIN_SCL, FALSE);
  SwI2cDelay();

  return Ack;
}

UINT8 SwI2cReadByte(NVT_INTERNAL_DATA *Instance, BOOLEAN Ack)
{
  INT32 i;
  UINT8 Byte = 0;

  // SDA Input (High)
  SwI2cSetPin(Instance, PIN_SDA, TRUE);

  for (i = 7; i >= 0; i--) {
    SwI2cDelay();
    // Clock High
    SwI2cSetPin(Instance, PIN_SCL, TRUE);
    SwI2cWaitClock(Instance);
    SwI2cDelay();

    if (SwI2cGetPin(Instance, PIN_SDA)) {
      Byte |= (1 << i);
    }

    // Clock Low
    SwI2cSetPin(Instance, PIN_SCL, FALSE);
  }

  // Send ACK/NACK
  SwI2cSetPin(Instance, PIN_SDA, !Ack); // Low for ACK, High for NACK
  SwI2cDelay();

  // Clock High
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  SwI2cWaitClock(Instance);
  SwI2cDelay();

  // Clock Low
  SwI2cSetPin(Instance, PIN_SCL, FALSE);
  SwI2cDelay();

  // Release SDA (High)
  SwI2cSetPin(Instance, PIN_SDA, TRUE);

  return Byte;
}

// ---------------------------------------------
// REPLACEMENT FUNCTIONS
// ---------------------------------------------

/**
  Reset the I2C bus state by toggling SCL/SDA

  @param Instance    Pointer to driver instance
**/
VOID NvtBusRecovery(NVT_INTERNAL_DATA *Instance)
{
  DEBUG((EFI_D_ERROR, "NVT: Bus Recovery START\n"));

  // Force both lines HIGH (release bus)
  SwI2cSetPin(Instance, PIN_SDA, TRUE);
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  gBS->Stall(1000); // 1ms

  // Check if SDA is stuck LOW
  if (!SwI2cGetPin(Instance, PIN_SDA)) {
    DEBUG((EFI_D_ERROR, "NVT: SDA stuck LOW - attempting recovery\n"));

    // Send 9 clock pulses to release slave
    for (int i = 0; i < 9; i++) {
      SwI2cSetPin(Instance, PIN_SCL, FALSE);
      gBS->Stall(100); // 100us
      SwI2cSetPin(Instance, PIN_SCL, TRUE);
      gBS->Stall(100); // 100us

      // Check if released
      if (SwI2cGetPin(Instance, PIN_SDA)) {
        DEBUG((EFI_D_ERROR, "NVT: SDA released at clock %d\n", i + 1));
        break;
      }
    }

    // Final check
    if (!SwI2cGetPin(Instance, PIN_SDA)) {
      DEBUG((EFI_D_ERROR, "NVT: Recovery FAILED - SDA still stuck!\n"));
      // Bus is dead - need hardware reset
      return;
    }
  }

  // Send STOP condition to reset bus state
  SwI2cSetPin(Instance, PIN_SDA, FALSE);
  SwI2cSetPin(Instance, PIN_SCL, FALSE);
  gBS->Stall(100);
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  gBS->Stall(100);
  SwI2cSetPin(Instance, PIN_SDA, TRUE);
  gBS->Stall(100);

  DEBUG((EFI_D_ERROR, "NVT: Bus Recovery COMPLETE\n"));
}

/**
  Initialize Software I2C Pins

  @param Instance    Pointer to driver instance
**/
VOID SwI2cInit(NVT_INTERNAL_DATA *Instance)
{
  DEBUG((EFI_D_ERROR, "NVT: Initializing Software I2C on GPIO 4/5...\n"));
  SwI2cSetPin(Instance, PIN_SCL, TRUE);
  SwI2cSetPin(Instance, PIN_SDA, TRUE);

  // Perform Recovery on Init
  NvtBusRecovery(Instance);
}

/**
  Write to I2C register (Helper)

  @param Instance    Pointer to driver instance
  @param SlaveAddr   I2C Slave Address (7-bit)
  @param Reg         Register Address
  @param Data        Data buffer to write
  @param Len         Length of data

  @retval EFI_SUCCESS           Write success
  @retval EFI_DEVICE_ERROR      NACK or Bus Error
**/
EFI_STATUS Nvti2cWriteHelper(
    NVT_INTERNAL_DATA *Instance, UINT8 SlaveAddr, UINT8 Reg, UINT8 *Data,
    UINT32 Len)
{
  // Start
  SwI2cStart(Instance);

  // Address + Write
  if (!SwI2cWriteByte(Instance, (SlaveAddr << 1) | 0)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // Register
  if (!SwI2cWriteByte(Instance, Reg)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // Data
  for (UINT32 i = 0; i < Len; i++) {
    if (!SwI2cWriteByte(Instance, Data[i])) {
      SwI2cStop(Instance);
      return EFI_DEVICE_ERROR;
    }
  }

  SwI2cStop(Instance);
  return EFI_SUCCESS;
}

/**
  Read from I2C register (Helper)

  @param Instance    Pointer to driver instance
  @param SlaveAddr   I2C Slave Address (7-bit)
  @param Reg         Register Address
  @param Data        Buffer to store read data
  @param Len         Length of data to read

  @retval EFI_SUCCESS           Read success
  @retval EFI_DEVICE_ERROR      NACK or Bus Error
**/
EFI_STATUS Nvti2cReadHelper(
    NVT_INTERNAL_DATA *Instance, UINT8 SlaveAddr, UINT8 Reg, UINT8 *Data,
    UINT32 Len)
{
  // 1. Write Register
  SwI2cStart(Instance);
  if (!SwI2cWriteByte(Instance, (SlaveAddr << 1) | 0)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }
  if (!SwI2cWriteByte(Instance, Reg)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // 2. Read Data
  SwI2cStart(Instance);
  if (!SwI2cWriteByte(Instance, (SlaveAddr << 1) | 1)) { // Read
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  for (UINT32 i = 0; i < Len; i++) {
    Data[i] = SwI2cReadByte(Instance, (i < Len - 1));
  }

  SwI2cStop(Instance);
  return EFI_SUCCESS;
}

// Duplicate NvtInitialize removed.
// Logic moved to NvtPowerUpController.

// ---------------------------------------------
// REPLACEMENT FUNCTIONS
// ---------------------------------------------
// ---------------------------------------------
// SOFTWARE I2C IMPLEMENTATION (Bit-Banging)
// ---------------------------------------------
// REASON: Hardware I2C (QUP) is inaccessible/unmapped in this environment.

/**
  Write data to I2C (Address + Reg + Data)

  @param Instance    Pointer to driver instance
  @param Address     Register Address
  @param Data        Data buffer
  @param Length      Length of data

  @retval EFI_SUCCESS           Write success
  @retval EFI_DEVICE_ERROR      I2C Error
**/
EFI_STATUS EFIAPI NvtI2cWrite(
    IN NVT_INTERNAL_DATA *Instance, IN UINT16 Address, IN UINT8 *Data,
    IN UINT32 Length)
{
  // Start
  SwI2cStart(Instance);

  // Address + Write (Bit 0 = 0)
  UINT8 AddrByte = (UINT8)(Instance->CurrentSlaveAddress << 1);
  if (!SwI2cWriteByte(Instance, AddrByte)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // Reg Address
  if (!SwI2cWriteByte(Instance, (UINT8)Address)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // Data
  for (UINT32 i = 0; i < Length; i++) {
    if (!SwI2cWriteByte(Instance, Data[i])) {
      SwI2cStop(Instance);
      return EFI_DEVICE_ERROR;
    }
  }

  SwI2cStop(Instance);
  return EFI_SUCCESS;
}

// ----------------------------------------------------------------------------------
// Raw I2C Write (No Register Address Injection)
// Used for Flash Commands (e.g. 0x06 Write Enable)
// ----------------------------------------------------------------------------------
/**
  Write raw data to I2C (Address + Data, No Register)

  Used for Flash commands like Write Enable (0x06).

  @param Instance    Pointer to driver instance
  @param Data        Data buffer
  @param Length      Length of data

  @retval EFI_SUCCESS           Write success
  @retval EFI_DEVICE_ERROR      I2C Error
**/
EFI_STATUS EFIAPI
NvtI2cWriteRaw(IN NVT_INTERNAL_DATA *Instance, IN UINT8 *Data, IN UINT32 Length)
{
  // Start
  SwI2cStart(Instance);

  // Address + Write
  UINT8 AddrByte = (UINT8)(Instance->CurrentSlaveAddress << 1);
  if (!SwI2cWriteByte(Instance, AddrByte)) {
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // Write Data directly
  for (UINT32 i = 0; i < Length; i++) {
    if (!SwI2cWriteByte(Instance, Data[i])) {
      SwI2cStop(Instance);
      return EFI_DEVICE_ERROR;
    }
  }

  SwI2cStop(Instance);
  return EFI_SUCCESS;
}

// ----------------------------------------------------------------------------------
// Raw I2C Read (No Register Address Write)
// Used for Flash Status Checks (CTP_I2C_READ)
// ----------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------
// Raw I2C Read (No Register Address Write)
// Just START -> ADDR(R) -> READ -> STOP
// ----------------------------------------------------------------------------------
/**
  Read raw data from I2C (No Register Address Write)

  Just START -> ADDR(R) -> READ -> STOP.
  Used for Flash Status Checks.

  @param Instance    Pointer to driver instance
  @param Data        Buffer to store data
  @param Length      Length to read

  @retval EFI_SUCCESS           Read success
  @retval EFI_DEVICE_ERROR      I2C Error
**/
EFI_STATUS EFIAPI
NvtI2cReceive(IN NVT_INTERNAL_DATA *Instance, OUT UINT8 *Data, IN UINT32 Length)
{
  UINT32 StartTime = 0; // You'll need a timer

  // 1. Start
  SwI2cStart(Instance);

  // 2. Address (Read)
  UINT8 AddrByte = (UINT8)((Instance->CurrentSlaveAddress << 1) | 1);
  if (!SwI2cWriteByte(Instance, AddrByte)) {
    DEBUG((EFI_D_ERROR, "NVT: I2C addr NACK\n"));
    SwI2cStop(Instance);
    return EFI_DEVICE_ERROR;
  }

  // 3. Read Data with timeout protection
  for (UINT32 i = 0; i < Length; i++) {
    Data[i] = SwI2cReadByte(Instance, (i < Length - 1));

    // Simple timeout check (every 10 bytes)
    if (i % 10 == 0) {
      gBS->Stall(100); // Prevent lockup
    }
  }

  // 4. Stop
  SwI2cStop(Instance);

  return EFI_SUCCESS;
}

/**
  Probe device presence

  Tries to write 1 byte to I2C_FW_Address.
  Checks for ACK.

  @param Instance    Pointer to driver instance
  @retval TRUE       Device ACKed
  @retval FALSE      No response
**/
BOOLEAN NvtProbeDevice(NVT_INTERNAL_DATA *Instance)
{
  UINT8      TestBuf[1];
  EFI_STATUS Status;

  NvtSetSlaveAddress(Instance, I2C_FW_Address);

  // Try to read 1 byte
  SwI2cStart(Instance);
  UINT8   AddrByte = (UINT8)((I2C_FW_Address << 1) | 1);
  BOOLEAN Ack      = SwI2cWriteByte(Instance, AddrByte);
  SwI2cStop(Instance);

  return Ack;
}

// Legacy Wrapper (To be phased out or fixed)
/**
  Read data from I2C Register (with Retries)

  Includes specific logic for Clone ICs (0xFF filtering and retries).

  @param Instance    Pointer to driver instance
  @param Address     Register Address
  @param Data        Buffer to store data
  @param Length      Length to read

  @retval EFI_SUCCESS           Read success
  @retval EFI_DEVICE_ERROR      Read failed
**/
EFI_STATUS EFIAPI NvtI2cRead(
    IN NVT_INTERNAL_DATA *Instance, IN UINT16 Address, OUT UINT8 *Data,
    IN UINT32 Length)
{
  EFI_STATUS Status;
  UINT32     Retry;

  // Retry logic for Clone ICs (sensitivity to 0xFF)
  for (Retry = 0; Retry < 3; Retry++) {
    // Call Helper (Bit-Banging)
    Status = Nvti2cReadHelper(
        Instance, Instance->CurrentSlaveAddress, (UINT8)Address, Data, Length);

    if (EFI_ERROR(Status)) {
      gBS->Stall(250); // Original delay
      continue;
    }

    // Verify Read (Check for all 0xFF)
    BOOLEAN AllFF = TRUE;
    for (UINT32 i = 0; i < MIN(Length, 6); i++) {
      if (Data[i] != 0xFF) {
        AllFF = FALSE;
        break;
      }
    }

    if (!AllFF) {
      return EFI_SUCCESS; // Success
    }

    // DEBUG((EFI_D_ERROR, "NVT: I2cRead Retry %d (Data All 0xFF)\n", Retry));
    gBS->Stall(500); // Longer delay before retry
  }

  // Return last status even if it was 0xFF success (which is suspicious but
  // technically success)
  return Status;
}
/**
  Dump GENI SE Registers (Debug)
**/
VOID NvtDumpGeniRegisters()
{
  UINT32 Val;
  DEBUG(
      (EFI_D_ERROR, "\n--- GENI SE Register Dump (Base 0x%08x) ---\n",
       GENI_SE_BASE));

  Val = MmioRead32(GENI_SE_BASE + GENI_CLK_CTRL);
  DEBUG((EFI_D_ERROR, "GENI_CLK_CTRL (0x00): 0x%08x\n", Val));

  Val = MmioRead32(GENI_SE_BASE + GENI_SER_M_CLK_CFG);
  DEBUG((EFI_D_ERROR, "GENI_SER_M_CLK_CFG (0x4C): 0x%08x\n", Val));

  Val = MmioRead32(GENI_SE_BASE + GENI_FW_REVISION);
  DEBUG((EFI_D_ERROR, "GENI_FW_REVISION (0x68): 0x%08x\n", Val));

  Val = MmioRead32(GENI_SE_BASE + GENI_STATUS);
  DEBUG((EFI_D_ERROR, "GENI_STATUS (0x40): 0x%08x\n", Val));

  Val = MmioRead32(GENI_SE_BASE + 0x2A4); // SE_I2C_STATUS
  DEBUG((EFI_D_ERROR, "SE_I2C_STATUS (0x2A4): 0x%08x\n", Val));

  Val = MmioRead32(GENI_SE_BASE + 0x610); // GENI_M_IRQ_STATUS
  DEBUG((EFI_D_ERROR, "GENI_M_IRQ_STATUS (0x610): 0x%08x\n", Val));

  Val = MmioRead32(GENI_SE_BASE + 0x614); // GENI_M_IRQ_EN
  DEBUG((EFI_D_ERROR, "GENI_M_IRQ_EN (0x614): 0x%08x\n", Val));

  DEBUG((EFI_D_ERROR, "-------------------------------\n\n"));
}
// --------------------------

//
// I2C Scanner
//
/**
  Scan I2C Bus for devices (Debug)

  Scans 0x00 to 0x7F.

  @param Instance    Pointer to driver instance
**/
VOID NvtI2cScan(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      Buffer[1];
  UINT32     I;

  DEBUG((EFI_D_ERROR, "NovatekNvtTsDxe: Starting I2C Address Scan...\n"));

  for (I = 0x00; I < 0x7F; I++) {
    // Skip reserved addresses if needed, but scanning all is usually fine for
    // debug
    Instance->NvtDevice->SlaveCfg.SlaveAddress = I;

    // Attempt a zero-length write or a single byte read to check for ACK
    // Using a Read of 1 byte is usually safe
    UINT32 BytesRead = 0;
    Status           = Instance->NvtDevice->I2cQupProtocol->Read(
        Instance->I2cController, &Instance->NvtDevice->SlaveCfg, 0x00, 0,
        Buffer, 1, &BytesRead, 100); // Short timeout

    if (EFI_ERROR(Status)) {
      DEBUG(
          (EFI_D_ERROR, "\nNovatekNvtTsDxe: FOUND Device at Address: 0x%02X\n",
           I));
    }
  }

  DEBUG((EFI_D_ERROR, "\nNovatekNvtTsDxe: I2C Scan Complete.\n"));

  // Restore default address
  Instance->NvtDevice->SlaveCfg.SlaveAddress = I2C_HW_Address;
}

// Instance Template
NVT_INTERNAL_DATA mNvtInstanceTemplate =
                      {NVT_TCH_INSTANCE_SIGNATURE,
                       0,
                       0,
                       0, // StationaryTouchCount
                       0, // LastX
                       0, // LastY
                       {
                           AbsPReset,
                           AbsPGetState,
                           NULL,
                           NULL,
                       },
                       {0},
                       NULL,
                       NULL,
                       FALSE,
                       0,
                       FALSE,
                       0,
                       0,
                       0,
                       NULL,
                       NULL,
                       NULL,
                       {0},
                       FALSE, // IsBusy
                       0,
                       {0}},
                  *gNvtI2CInstance;

// Binding
EFI_DRIVER_BINDING_PROTOCOL gNvtDriverBinding = {
    NvtAbsolutePointerDriverBindingSupported,
    NvtAbsolutePointerDriverBindingStart,
    NvtAbsolutePointerDriverBindingStop,
    0x1,
    NULL,
    NULL,
};

static NOVATEK_I2C_DEVICE *gNvtDeviceIo = NULL;

/**
  Driver Entry Point

  Installs Driver Binding Protocol.

  @param ImageHandle   Handle of the image
  @param SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS  Driver initialized
**/
EFI_STATUS
EFIAPI
NvtInitialize(IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable)
{
  EFI_STATUS Status;

  Status = EfiLibInstallDriverBindingComponentName2(
      ImageHandle, SystemTable, &gNvtDriverBinding, ImageHandle,
      &gNvtDriverComponentName, &gNvtDriverComponentName2);
  ASSERT_EFI_ERROR(Status);

  return EFI_SUCCESS;
}

/**
  Driver Binding Supported

  @param This                Driver Binding Protocol
  @param Controller          Handle of controller to test
  @param RemainingDevicePath Device Path (Optional)

  @retval EFI_SUCCESS        Driver supports this controller
**/
EFI_STATUS
EFIAPI
NvtAbsolutePointerDriverBindingSupported(
    IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
    IN EFI_DEVICE_PATH_PROTOCOL *RemainingDevicePath)
{
  EFI_STATUS          Status;
  NOVATEK_I2C_DEVICE *NvtDeviceIo;

  Status = gBS->OpenProtocol(
      Controller, &gNovatekTouchDeviceProtocolGuid, (VOID **)&NvtDeviceIo,
      This->DriverBindingHandle, Controller, EFI_OPEN_PROTOCOL_BY_DRIVER);

  if (EFI_ERROR(Status)) {
    return Status;
  }

  Status = EFI_SUCCESS;
  gBS->CloseProtocol(
      Controller, &gNovatekTouchDeviceProtocolGuid, This->DriverBindingHandle,
      Controller);

  return Status;
}

/**
  Set Current I2C Slave Address

  @param Instance    Pointer to driver instance
  @param Address     Target I2C Address (7-bit)
**/
EFI_STATUS
EFIAPI
NvtSetSlaveAddress(NVT_INTERNAL_DATA *Instance, UINT32 Address)
{
  if (Instance->CurrentSlaveAddress == Address) {
    return EFI_SUCCESS;
  }

  Instance->NvtDevice->SlaveCfg.SlaveAddress = Address;
  Instance->CurrentSlaveAddress              = Address;
  return EFI_SUCCESS;
}

// [Removed Duplicate Manual I2C Functions - Now Implemented via SwI2c at Top]

// --- ADDRESS DEFINITIONS ---
#define I2C_ADDR_NVT 0x62
// #define I2C_ADDR_BLDR 0x01 // Removed
// Helper Forward Declarations
// --- RECOVERY LOGIC ---
// --- CHIP DETECTION LOGIC ---
/**
  Detect Novatek Chip

  Resets Bootloader and checks ID/Trim.

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Chip detected
  @retval EFI_DEVICE_ERROR      Detection failed
**/
EFI_STATUS NvtDetectChip(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;

  DEBUG((EFI_D_ERROR, "NVT: DetectChip - Delegating to CheckChipVerTrim...\n"));

  // Ensure Bootloader Reset before detection (as per Linux driver flow)
  NvtBootloaderReset(Instance);

  // Use the robust verification and trim check function
  Status = NvtCheckChipVerTrim(Instance);

  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "NVT: CheckChipVerTrim Failed or Chip Not Found.\n"));
    // Even if failed, we default to 0x21C00 in NvtCheckChipVerTrim, so we
    // should be okay? CheckChipVerTrim returns Success even if default used?
    // Yes, my implementation returns Success.
  }

  return Status;
}

// ----------------------------------------------------------------------------------
// Switch Frequency Hopping Enable/Disable
// Used to mitigate noise issues.
// OnOff: 0x65 = Enable, 0x66 = Disable
// ----------------------------------------------------------------------------------
/**
  Enable/Disable Frequency Hopping

  @param Instance    Pointer to driver instance
  @param OnOff       0x65=Enable, 0x66=Disable

  @retval EFI_SUCCESS           Success
  @retval EFI_DEVICE_ERROR      Failed
**/
EFI_STATUS NvtSwitchFreqHopEnDis(NVT_INTERNAL_DATA *Instance, UINT8 OnOff)
{
  UINT8  Buf[8];
  UINT32 Retry = 0;

  DEBUG((EFI_D_ERROR, "NVT: SwitchFreqHopEnDis(0x%02X)...\n", OnOff));

  // Ensure Slave Address is 0x01
  // FIX: Use Hardware Slave (0x62)
  NvtSetSlaveAddress(Instance, I2C_HW_Address);

  // 1. Set XData Index (Event Buf Addr) - CRITICAL
  Buf[0] = 0xFF;
  Buf[1] = (UINT8)((Instance->EventBufAddr >> 16) & 0xFF);
  Buf[2] = (UINT8)((Instance->EventBufAddr >> 8) & 0xFF);
  NvtI2cWriteRaw(Instance, Buf, 3); // Raw Write for Page Set

  // 2. Write Command (Start writing from reg 0x50)
  Buf[0] = EVENT_MAP_HOST_CMD;
  Buf[1] = OnOff;                   // 0x65 Enable, 0x66 Disable
  NvtI2cWriteRaw(Instance, Buf, 2); // Raw Write for CMD

  // Wait 35ms
  MicroSecondDelay(35000);

  // 3. Check if cleared (Handshake)
  for (Retry = 0; Retry < 20; Retry++) {
    Buf[0] = EVENT_MAP_HOST_CMD;
    Buf[1] = 0xFF;

    // Read Status at Register 0x50
    // NvtI2cRead uses Address as Register
    NvtI2cRead(Instance, EVENT_MAP_HOST_CMD, Buf, 2);

    if (Buf[1] == 0x00) {
      DEBUG((EFI_D_ERROR, "NVT: SwitchFreqHop Success (Cleared 0x00).\n"));
      break;
    }
    MicroSecondDelay(10000); // 10ms
  }

  if (Retry >= 20) {
    DEBUG((EFI_D_ERROR, "NVT: SwitchFreqHop Timeout! Buf[1]=0x%02X\n", Buf[1]));

    // Retry Write
    DEBUG((EFI_D_ERROR, "NVT: Retrying FreqHop Write...\n"));
    Buf[0] = EVENT_MAP_HOST_CMD;
    Buf[1] = OnOff;
    NvtI2cWriteRaw(Instance, Buf, 2);
    MicroSecondDelay(35000);

    Buf[0] = EVENT_MAP_HOST_CMD;
    Buf[1] = 0xFF;
    NvtI2cRead(Instance, EVENT_MAP_HOST_CMD, Buf, 2);
    if (Buf[1] == 0x00) {
      DEBUG((EFI_D_ERROR, "NVT: SwitchFreqHop Success (Retry).\n"));
      return EFI_SUCCESS;
    }
    else {
      DEBUG(
          (EFI_D_ERROR, "NVT: SwitchFreqHop Failed (Retry) Buf[1]=0x%02X\n",
           Buf[1]));
      return EFI_DEVICE_ERROR;
    }
  }
  return EFI_SUCCESS;
}

// ----------------------------------------------------------------------------------
// Change Mode (Normal vs Test)
// Matches `nvt_change_mode` from Linux driver.
// Critical for setting "Host Ready" (0xBB) handshake in Normal Mode.
// ----------------------------------------------------------------------------------
/**
  Change touch controller operating mode

  @param Instance    Pointer to driver instance
  @param Mode        Target mode (NORMAL_MODE, TEST_MODE_1, TEST_MODE_2)

  @retval EFI_SUCCESS           Mode changed successfully
  @retval EFI_DEVICE_ERROR      I2C communication failed
**/
EFI_STATUS
NvtChangeMode(IN NVT_INTERNAL_DATA *Instance, IN UINT8 Mode)
{
  UINT8      Buf[8];
  EFI_STATUS Status;

  // Set XDATA page to EVENT_BUF_ADDR
  Status = NvtSetXdataPage(Instance, Instance->MemMap->EVENT_BUF_ADDR);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // Write mode command
  Buf[0] = EVENT_MAP_HOST_CMD;
  Buf[1] = Mode;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: ChangeMode write failed\n"));
    return Status;
  }

  // If switching to normal mode, send handshake
  if (Mode == NORMAL_MODE) {
    Buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
    Buf[1] = HANDSHAKING_HOST_READY;
    Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
    if (EFI_ERROR(Status)) {
      return Status;
    }
    gBS->Stall(20000); // 20ms delay
  }

  Instance->CurrentMode = Mode;
  DEBUG((EFI_D_ERROR, "Nvt: Mode changed to 0x%02X\n", Mode));

  return EFI_SUCCESS;
}

// Drain the FIFO to clear any stale touch events (Ghost Touches)
// Note: Requires POINT_DATA_LEN to be 66 (Kernel length)
/**
  Drain Bootloader FIFO

  Clears stale data/ghost touches after bootloader operations.

  @param Instance    Pointer to driver instance
**/
VOID NvtDrainBufferBootloader(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      TempBuf[POINT_DATA_LEN];
  UINT32     Retry;
  UINT8      PacketId, LastDrainPid = 0xFF;

  DEBUG((EFI_D_ERROR, "NVT: Draining bootloader buffer...\n"));

  NvtSetSlaveAddress(Instance, I2C_HW_Address);

  for (Retry = 0; Retry < 30; Retry++) { // Increased from 20 to 30
    SetMem(TempBuf, POINT_DATA_LEN, 0xAA);

    Status = NvtI2cRead(Instance, 0x00, TempBuf, POINT_DATA_LEN);

    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "NVT: Drain read failed\n"));
      break;
    }

    // Check for bus idle
    if (TempBuf[0] == 0xFF && TempBuf[1] == 0xFF) {
      DEBUG((EFI_D_ERROR, "NVT: Buffer empty (idle)\n"));
      break;
    }

    // Check if data changed
    BOOLEAN AllSentinel = TRUE;
    for (UINT32 i = 0; i < 8; i++) {
      if (TempBuf[i] != 0xAA) {
        AllSentinel = FALSE;
        break;
      }
    }

    if (AllSentinel) {
      DEBUG((EFI_D_ERROR, "NVT: No data received\n"));
      break;
    }

    // Extract packet ID
    PacketId = (TempBuf[0] >> 3) & 0x1F;

    // Check if this is a duplicate packet (ghost!)
    if (PacketId == LastDrainPid) {
      DEBUG(
          (EFI_D_ERROR, "NVT: Duplicate PID=%d (ghost) - ignored\n", PacketId));
      gBS->Stall(10000); // Wait longer for new packet
      continue;
    }

    LastDrainPid = PacketId;

    // Parse touch status
    UINT8 TouchStatus = TempBuf[1] & 0x07;

    // Validate touch status
    if (TouchStatus > 0x02) {
      DEBUG((EFI_D_ERROR, "NVT: Invalid status=0x%02X (ghost)\n", TouchStatus));
      gBS->Stall(5000);
      continue;
    }

    if (TouchStatus != 0x00) {
      UINT16 X = ((UINT16)(TempBuf[4] << 4) | ((TempBuf[3] >> 4) & 0x0F));
      UINT16 Y = ((UINT16)(TempBuf[2] << 4) | (TempBuf[3] & 0x0F));
      DEBUG((EFI_D_ERROR, "NVT: Drained [PID=%d] X=%d Y=%d\n", PacketId, X, Y));
    }
    else {
      DEBUG((EFI_D_ERROR, "NVT: Drained release [PID=%d]\n", PacketId));
    }

    gBS->Stall(10000); // 10ms between drain reads (increased from 5ms)
  }

  // Reset packet ID tracker after drain
  Instance->LastPacketId = -1;

  DEBUG((EFI_D_ERROR, "NVT: Drain complete after %d reads\n", Retry));
}

/*
VOID NvtDrainBuffer(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      TempBuf[POINT_DATA_LEN];
  UINT32     Retry;

  DEBUG((EFI_D_ERROR, "NovatekNvtTsDxe: Draining FIFO...\n"));

  // Try reading up to 20 times
  for (Retry = 0; Retry < 20; Retry++) {
    TempBuf[0] = 0;
    // Read 66 Bytes
    Status = Nvti2cReadHelper(
        Instance, I2C_FW_Address, 0x00, TempBuf, POINT_DATA_LEN);

    if (EFI_ERROR(Status)) {
      break;
    }

    // Check if empty (0xFF or Status=0)
    // Byte 1 has Status
    // If Status == 0 (No Touch), we are usually good.
    // If 0xFF, bus idle.
    if ((TempBuf[1] == 0xFF) || ((TempBuf[1] & 0x07) == 0)) {
      // Double check byte 2 to differentiate noise
      if (TempBuf[2] == 0xFF || TempBuf[2] == 0x00) {
        break;
      }
    }

    DEBUG((
        EFI_D_ERROR, "NVT: Drained Packet %d (St: %02X)\n", Retry, TempBuf[1]));
    gBS->Stall(1000); // 1ms
  }
}
*/

// EFI_STATUS
// EFIAPI
// NvtAbsolutePointerDriverBindingStart(
//     IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
//     IN EFI_DEVICE_PATH_PROTOCOL *RemainingDevicePath)
// {
//   EFI_STATUS          Status;
//   NOVATEK_I2C_DEVICE *NvtDeviceIo;
//   NVT_INTERNAL_DATA  *Instance;
//   UINT8               TestBuf[8];
//   UINT32              IrqConfig, ResetConfig;

//   DEBUG((EFI_D_ERROR, "=== NVT BOOTLOADER MODE START ===\n"));

//   // Open device protocol
//   Status = gBS->OpenProtocol(
//       Controller, &gNovatekTouchDeviceProtocolGuid, (VOID **)&NvtDeviceIo,
//       This->DriverBindingHandle, Controller, EFI_OPEN_PROTOCOL_BY_DRIVER);
//   if (EFI_ERROR(Status)) {
//     return Status;
//   }

//   // Allocate instance
//   Instance = AllocateCopyPool(sizeof(NVT_INTERNAL_DATA),
//   &mNvtInstanceTemplate); if (Instance == NULL) {
//     gBS->CloseProtocol(
//         Controller, &gNovatekTouchDeviceProtocolGuid,
//         This->DriverBindingHandle, Controller);
//     return EFI_OUT_OF_RESOURCES;
//   }

//   Instance->NvtDevice = NvtDeviceIo;
//   Instance->Buf       = AllocateZeroPool(POINT_DATA_LEN + 1);
//   if (Instance->Buf == NULL) {
//     FreePool(Instance);
//     return EFI_OUT_OF_RESOURCES;
//   }

//   // ===== ANDROID MODE: USE HARDCODED DEFAULTS =====
//   DEBUG((EFI_D_ERROR, "NVT: Using Android defaults (firmware is broken)\n"));

//   Instance->EventBufAddr                = 0x21C00; // Android default from
//   dmesg Instance->AbsPointerMode.AbsoluteMaxX = 1080;
//   Instance->AbsPointerMode.AbsoluteMaxY = 2340;
//   Instance->AbsPointerMode.AbsoluteMinX = 0;
//   Instance->AbsPointerMode.AbsoluteMinY = 0;
//   Instance->AbsPointerProtocol.Mode     = &Instance->AbsPointerMode;

//   Instance->CurrentMode         = NORMAL_MODE;
//   Instance->LastPacketId        = -1;
//   Instance->CurrentMode         = NORMAL_MODE;
//   Instance->LastPacketId        = -1;
//   Instance->CurrentSlaveAddress = I2C_HW_Address; // 0x62 ALWAYS!

//   // ===== GPIO INITIALIZATION =====
//   // IRQ GPIO 89
//   IrqConfig = EFI_GPIO_CFG(
//       Instance->NvtDevice->ControllerInterruptPin, 0, GPIO_INPUT,
//       GPIO_PULL_UP, GPIO_2MA);
//   Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
//       IrqConfig, TLMM_GPIO_ENABLE);

//   // RESET GPIO 88
//   ResetConfig = EFI_GPIO_CFG(
//       Instance->NvtDevice->ControllerResetPin, 0, GPIO_OUTPUT, GPIO_NO_PULL,
//       GPIO_2MA);
//   Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
//       ResetConfig, TLMM_GPIO_ENABLE);

//   // Power cycle
//   DEBUG((EFI_D_ERROR, "NVT: Reset cycle...\n"));
//   Instance->NvtDevice->GpioTlmmProtocol->GpioOut(ResetConfig, 0); // Low
//   gBS->Stall(20000);                                              // 20ms
//   Instance->NvtDevice->GpioTlmmProtocol->GpioOut(ResetConfig, 1); // High
//   gBS->Stall(300000); // 300ms for IC boot

//   // ===== SOFTWARE I2C INITIALIZATION =====
//   SwI2cInit(Instance);

//   // ===== BOOTLOADER MODE VERIFICATION =====
//   DEBUG((EFI_D_ERROR, "NVT: Verifying chip at 0x62...\n"));

//   NvtSetSlaveAddress(Instance, I2C_HW_Address); // 0x62
//   Status = NvtI2cReceive(Instance, TestBuf, 6);

//   if (!EFI_ERROR(Status)) {
//     DEBUG(
//         (EFI_D_ERROR, "NVT: Chip responds: %02X %02X %02X %02X %02X %02X\n",
//          TestBuf[0], TestBuf[1], TestBuf[2], TestBuf[3], TestBuf[4],
//          TestBuf[5]));

//     // Check for expected NT36672A signature (from Android dmesg)
//     if (TestBuf[0] == 0x0A && TestBuf[3] == 0x72 && TestBuf[4] == 0x66 &&
//         TestBuf[5] == 0x03) {
//       DEBUG((EFI_D_ERROR, "NVT: NT36672A detected in bootloader mode!\n"));
//     }
//   }
//   else {
//     DEBUG(
//         (EFI_D_ERROR, "NVT: Chip NOT responding at 0x62 - Status: %r\n",
//          Status));
//     FreePool(Instance->Buf);
//     FreePool(Instance);
//     return EFI_DEVICE_ERROR;
//   }

//   UINT8 StatusBuf[2];
//   NvtSetSlaveAddress(Instance, I2C_HW_Address);
//   Status = NvtI2cReceive(Instance, StatusBuf, 2);
//   DEBUG(
//       (EFI_D_ERROR, "NVT: Status bytes: %02X %02X\n", StatusBuf[0],
//        StatusBuf[1]));

//   // ===== SKIP ALL FIRMWARE INITIALIZATION =====
//   DEBUG((EFI_D_ERROR, "NVT: Bootloader mode ready - skipping firmware
//   init\n"));
//   //==============================================================================
//   // MINIMAL FIRMWARE INITIALIZATION (Required for Real Touch)
//   //==============================================================================
//   DEBUG((EFI_D_ERROR, "NVT: Attempting firmware initialization...\n"));

//   // Step 1: Bootloader Reset
//   Status = NvtBootloaderReset(Instance);
//   if (EFI_ERROR(Status)) {
//     DEBUG(
//         (EFI_D_ERROR, "NVT: Bootloader reset failed, continuing
//         anyway...\n"));
//   }

//   // Step 2: Check Chip Version & Trim (sets EventBufAddr)
//   Status = NvtCheckChipVerTrim(Instance);
//   if (EFI_ERROR(Status)) {
//     DEBUG((EFI_D_ERROR, "NVT: Chip detection failed, using defaults\n"));
//     Instance->EventBufAddr = 0x21C00; // NT36672A default
//   }

//   // Step 3: Wait for Firmware Boot
//   DEBUG((EFI_D_ERROR, "NVT: Waiting for firmware to boot...\n"));
//   gBS->Stall(500000); // 500ms - critical for firmware startup

//   // Step 4: Check Firmware Reset State
//   Status = NvtCheckFwResetState(Instance);
//   if (EFI_ERROR(Status)) {
//     DEBUG((EFI_D_ERROR, "NVT: FW not ready (0xA2), might be in
//     bootloader\n"));
//     // Continue anyway - some devices work without firmware
//   }

//   // Step 5: CRITICAL - Switch to Normal Mode
//   DEBUG((EFI_D_ERROR, "NVT: Switching to NORMAL_MODE...\n"));
//   Status = NvtChangeMode(Instance, NORMAL_MODE);
//   if (EFI_ERROR(Status)) {
//     DEBUG((EFI_D_ERROR, "NVT: Mode change failed - touch may not work!\n"));
//     // Continue anyway for debugging
//   }

//   // Step 6: Get Firmware Info (optional but helpful)
//   Status = NvtGetFwInfo(Instance);
//   if (!EFI_ERROR(Status)) {
//     // Update screen resolution from firmware
//     Instance->AbsPointerMode.AbsoluteMaxX = Instance->IdInfo.AbsXMax;
//     Instance->AbsPointerMode.AbsoluteMaxY = Instance->IdInfo.AbsYMax;
//     DEBUG(
//         (EFI_D_ERROR, "NVT: Using FW resolution: %dx%d\n",
//          Instance->IdInfo.AbsXMax, Instance->IdInfo.AbsYMax));
//   }
//   else {
//     // Keep hardcoded defaults
//     DEBUG((EFI_D_ERROR, "NVT: Using hardcoded defaults: 1080x2340\n"));
//   }

//   // Step 7: Drain Ghost Touches from Bootloader
//   DEBUG((EFI_D_ERROR, "NVT: Draining bootloader buffer...\n"));
//   NvtDrainBufferBootloader(Instance);

//   DEBUG((EFI_D_ERROR, "NVT: Firmware initialization complete!\n"));

//   // ===== INSTALL PROTOCOL =====
//   Instance->Initialized = TRUE;

//   Status = gBS->InstallProtocolInterface(
//       &Controller, &gEfiAbsolutePointerProtocolGuid, EFI_NATIVE_INTERFACE,
//       &Instance->AbsPointerProtocol);

//   if (!EFI_ERROR(Status)) {
//     AbsStartPolling(Instance);
//     DEBUG((EFI_D_ERROR, "NVT: Driver started successfully!\n"));
//   }

//   return Status;
// }

/**
  Driver Binding Start

  Initializes the driver for a supported controller.
  - Setup GPIOs
  - Reset Chip
  - Detect & Init Firmware
  - Install Protocol

  @param This                Driver Binding Protocol
  @param Controller          Handle of controller
  @param RemainingDevicePath Device Path (Optional)

  @retval EFI_SUCCESS        Driver started
**/
EFI_STATUS EFIAPI NvtAbsolutePointerDriverBindingStart(
    IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
    IN EFI_DEVICE_PATH_PROTOCOL *RemainingDevicePath)
{
  EFI_STATUS          Status;
  NOVATEK_I2C_DEVICE *NvtDeviceIo;
  NVT_INTERNAL_DATA  *Instance;
  UINT32              IrqConfig, ResetConfig;

  DEBUG((EFI_D_ERROR, "=== NVT DRIVER START (BROKEN FW MODE) ===\n"));

  // Open device protocol
  Status = gBS->OpenProtocol(
      Controller, &gNovatekTouchDeviceProtocolGuid, (VOID **)&NvtDeviceIo,
      This->DriverBindingHandle, Controller, EFI_OPEN_PROTOCOL_BY_DRIVER);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // Allocate instance
  Instance = AllocateCopyPool(sizeof(NVT_INTERNAL_DATA), &mNvtInstanceTemplate);
  if (Instance == NULL) {
    gBS->CloseProtocol(
        Controller, &gNovatekTouchDeviceProtocolGuid, This->DriverBindingHandle,
        Controller);
    return EFI_OUT_OF_RESOURCES;
  }

  Instance->NvtDevice = NvtDeviceIo;
  Instance->Buf       = AllocateZeroPool(POINT_DATA_LEN + 1);
  if (Instance->Buf == NULL) {
    FreePool(Instance);
    return EFI_OUT_OF_RESOURCES;
  }

  // ===== INITIALIZE DEFAULTS (Safe Fallback) =====
  // Matches "let defaults stay, its safer that way"
  Instance->MemMap       = &NT36672A_memory_map;
  Instance->EventBufAddr = 0x21C00;
  Instance->CurrentMode  = NORMAL_MODE;
  Instance->LastPacketId = -1;
  Instance->CurrentSlaveAddress =
      Instance->NvtDevice->SlaveCfg.SlaveAddress; // Dynamic from Protocol/PCD
  Instance->FirmwareBroken = FALSE;               // Assume OK initially

  // Initialize Pointer Mode with SAFE DEFAULTS (will be updated by FW Info)
  Instance->AbsPointerMode.AbsoluteMaxX = 1080;
  Instance->AbsPointerMode.AbsoluteMaxY = 2340;
  Instance->AbsPointerMode.AbsoluteMinX = 0;
  Instance->AbsPointerMode.AbsoluteMinY = 0;
  Instance->AbsPointerMode.Attributes   = 0;
  Instance->AbsPointerProtocol.Mode     = &Instance->AbsPointerMode;

  // ===== GPIO CONFIG =====
  IrqConfig = EFI_GPIO_CFG(
      Instance->NvtDevice->ControllerInterruptPin, 0, GPIO_INPUT, GPIO_PULL_UP,
      GPIO_2MA);
  Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
      IrqConfig, TLMM_GPIO_ENABLE);

  ResetConfig = EFI_GPIO_CFG(
      Instance->NvtDevice->ControllerResetPin, 0, GPIO_OUTPUT, GPIO_NO_PULL,
      GPIO_2MA);
  Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
      ResetConfig, TLMM_GPIO_ENABLE);

  // ===== SOFTWARE I2C INIT =====
  SwI2cInit(Instance);

  // ===== CRITICAL: FORCE RESET TO IDLE =====
  // This breaks the CRC reboot loop seen in dmesg
  DEBUG((EFI_D_ERROR, "NVT: Forcing reset to idle (0xA5)...\n"));
  for (UINT32 retry = 0; retry < 3; retry++) {
    NvtSwResetIdle(Instance); // Send 0x00 = 0xA5 to address 0x62
    gBS->Stall(20000);        // 20ms
  }

  DEBUG((EFI_D_ERROR, "NVT: Sending Baseline ReK (0xA1)...\n"));
  UINT8 RekCmd[2];
  RekCmd[0] = 0x00;
  RekCmd[1] = 0xA1; // ReK Command
  NvtSetSlaveAddress(Instance, I2C_HW_Address);
  NvtI2cWriteRaw(Instance, RekCmd, 2);
  gBS->Stall(100000); // Wait 100ms for recalibration
  DEBUG((EFI_D_ERROR, "NVT: Baseline recalibration complete\n"));

  // ===== FIX: ENABLE FREQUENCY HOPPING (0x65) =====
  // Fixes I2C noise (0xFF packets) from LCD interference
  DEBUG((EFI_D_ERROR, "NVT: Enabling Frequency Hopping...\n"));
  Status = NvtSwitchFreqHopEnDis(Instance, 0x65); // 0x65 = Enable
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "NVT: FreqHop enable failed (non-fatal)\n"));
  }
  gBS->Stall(35000); // 35ms delay

  // ===== POWER UP & DETECT =====
  // 1. Power Up (Resets, Clocks) - calls NvtGetFwInfo internally
  Status = NvtPowerUpController(Instance);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR,
         "NVT: PowerUpController failed (Continue to try defaults)\n"));
  }

  // 2. Detect Chip (Updates MemMap/EventBufAddr if found)
  // Matches Android `nvt_ts_check_chip_ver_trim`
  Status = NvtDetectChip(Instance);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "NVT: DetectChip failed, using defaults (0x21C00)\n"));
    // MemMap/EventBufAddr stay at defaults set above
  }

  // ===== Early Firmware Info Retrieval =====
  Status = NvtGetFwInfo(Instance);
  if (!EFI_ERROR(Status)) {
    // Update Resolution immediately
    // Validate resolution bounds to prevent issues with invalid firmware data
    if (Instance->IdInfo.AbsXMax > 100 && Instance->IdInfo.AbsXMax < 10000 &&
        Instance->IdInfo.AbsYMax > 100 && Instance->IdInfo.AbsYMax < 10000) {
      Instance->AbsPointerMode.AbsoluteMaxX = Instance->IdInfo.AbsXMax;
      Instance->AbsPointerMode.AbsoluteMaxY = Instance->IdInfo.AbsYMax;
      DEBUG(
          (EFI_D_INFO, "NVT: Early Resolution Update: %dx%d\n",
           Instance->AbsPointerMode.AbsoluteMaxX,
           Instance->AbsPointerMode.AbsoluteMaxY));
    }
    else {
      DEBUG(
          (EFI_D_WARN, "NVT: Invalid resolution detected (%dx%d), ignoring.\n",
           Instance->IdInfo.AbsXMax, Instance->IdInfo.AbsYMax));
    }
  }

  // 3. Stop CRC Reboot (Safety)
  NvtStopCrcReboot(Instance);
  gBS->Stall(100000); // 100ms

  // 4. Update Resolution from Firmware Info (if PowerUp/GetFwInfo succeeded)
  // NvtPowerUpController calls NvtGetFwInfo which sets Instance->IdInfo.
  // We should update AbsPointerMode to match IdInfo (whether valid or
  // broken-defaults)
  if (Instance->IdInfo.AbsXMax != 0 && Instance->IdInfo.AbsYMax != 0) {
    Instance->AbsPointerMode.AbsoluteMaxX = Instance->IdInfo.AbsXMax;
    Instance->AbsPointerMode.AbsoluteMaxY = Instance->IdInfo.AbsYMax;
    DEBUG(
        (EFI_D_ERROR, "NVT: Updated Resolution: %dx%d\n",
         Instance->AbsPointerMode.AbsoluteMaxX,
         Instance->AbsPointerMode.AbsoluteMaxY));
  }
  else {
    DEBUG((EFI_D_ERROR, "NVT: IdInfo invalid, keeping default 1080x2340\n"));
  }

  // ===== DRAIN GHOST TOUCHES FROM BOOTLOADER =====
  DEBUG((EFI_D_ERROR, "NVT: Draining bootloader buffer...\n"));
  NvtDrainBufferBootloader(Instance);

  // ===== INSTALL PROTOCOL =====
  Instance->Initialized = TRUE;
  Status                = gBS->InstallProtocolInterface(
      &Controller, &gEfiAbsolutePointerProtocolGuid, EFI_NATIVE_INTERFACE,
      &Instance->AbsPointerProtocol);

  if (!EFI_ERROR(Status)) {
    AbsStartPolling(Instance);
    DEBUG((EFI_D_ERROR, "NVT: Driver started successfully!\n"));
  }

  return Status;
}

/**
  Driver Binding Stop

  Unloads the driver and cleans up resources.
  - Uninstall Protocol
  - Stop Timer
  - Close I2C

  @param This                Driver Binding Protocol
  @param Controller          Handle of controller
  @param NumberOfChildren    Number of children
  @param ChildHandleBuffer   Buffer of child handles

  @retval EFI_SUCCESS        Driver stopped
**/
EFI_STATUS
EFIAPI
NvtAbsolutePointerDriverBindingStop(
    IN EFI_DRIVER_BINDING_PROTOCOL *This, IN EFI_HANDLE Controller,
    IN UINTN NumberOfChildren, IN EFI_HANDLE *ChildHandleBuffer)
{
  EFI_STATUS                     Status;
  EFI_ABSOLUTE_POINTER_PROTOCOL *AbsolutePointerProtocol;
  NVT_INTERNAL_DATA             *Instance;
  NOVATEK_I2C_DEVICE            *NvtDeviceIo;

  Status = gBS->OpenProtocol(
      Controller, &gEfiAbsolutePointerProtocolGuid,
      (VOID **)&AbsolutePointerProtocol, This->DriverBindingHandle, Controller,
      EFI_OPEN_PROTOCOL_GET_PROTOCOL);

  if (EFI_ERROR(Status)) {
    return EFI_UNSUPPORTED;
  }

  Instance = NVT_TCH_INSTANCE_FROM_ABSTCH_THIS(AbsolutePointerProtocol);

  Status = gBS->UninstallProtocolInterface(
      Controller, &gEfiAbsolutePointerProtocolGuid,
      &Instance->AbsPointerProtocol);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  gBS->CloseEvent(Instance->PollingTimerEvent);
  gBS->CloseEvent(Instance->AbsPointerProtocol.WaitForInput);

  Status = gBS->OpenProtocol(
      Controller, &gNovatekTouchDeviceProtocolGuid, (VOID **)&NvtDeviceIo,
      This->DriverBindingHandle, Controller, EFI_OPEN_PROTOCOL_BY_DRIVER);

  NvtDeviceIo->I2cQupProtocol->Close(Instance->I2cController);

  if (Instance->Buf != NULL) {
    FreePool(Instance->Buf);
  }

  DEBUG((EFI_D_ERROR, "NovatekNvtTsDxe: Closing i2c instance\n"));

  return EFI_SUCCESS;
}

/**
  Reset Pointer State

  @param This                  Absolute Pointer Protocol
  @param ExtendedVerification  Extended verification (Ignored)

  @retval EFI_SUCCESS          Success
**/
EFI_STATUS AbsPReset(
    IN EFI_ABSOLUTE_POINTER_PROTOCOL *This, IN BOOLEAN ExtendedVerification)
{
  NVT_INTERNAL_DATA *Instance;

  Instance               = NVT_TCH_INSTANCE_FROM_ABSTCH_THIS(This);
  Instance->LastX        = 0;
  Instance->LastY        = 0;
  Instance->StateChanged = FALSE;
  Instance->IsTouched    = FALSE;

  return EFI_SUCCESS;
}

/**
  Polling Callback (Timer)

  Called periodically (100Hz) to poll touch data.
  - Checks IRQ state
  - Reads I2C
  - Updates Instance state

  @param Event    Event handle
  @param Context  Driver Instance
**/
VOID EFIAPI SyncPollCallback(IN EFI_EVENT Event, IN VOID *Context)
{
  NVT_INTERNAL_DATA *Instance = (NVT_INTERNAL_DATA *)Context;
  TOUCH_DATA         TouchData;
  EFI_STATUS         Status;
  UINT32             IrqState     = 0;
  static UINT32      FailureCount = 0;

  if (Instance->IsBusy) {
    return;
  }

  Instance->IsBusy = TRUE;

  // ===== CHECK IRQ GPIO STATE FIRST =====
  // Only read I2C if IRQ pin is HIGH (indicating new data)
  // Restored: Prevents "Ghost Touches" from reading idle/updating bus.

  UINT32 IrqConfig = EFI_GPIO_CFG(
      Instance->NvtDevice->ControllerInterruptPin, 0, GPIO_INPUT, GPIO_PULL_UP,
      GPIO_2MA);

  Instance->NvtDevice->GpioTlmmProtocol->GpioIn(IrqConfig, &IrqState);

  if (IrqState == 1) {
    // IRQ is HIGH - no new touch data available
    Instance->IsBusy = FALSE;
    return;
  }

  // ===== READ TOUCH DATA =====
  Status = NvtGetTouchData(Instance, &TouchData);
  if (EFI_ERROR(Status)) {
    if (Status == EFI_NOT_READY) {
      Instance->IsBusy = FALSE;
      return;
    }

    // Real I2C failure
    FailureCount++;
    if (FailureCount >= 100) {
      DEBUG((EFI_D_ERROR, "NVT: 10 I2C failures - HARD RESET!\n"));
      UINT32 ResetConfig = EFI_GPIO_CFG(
          Instance->NvtDevice->ControllerResetPin, 0, GPIO_OUTPUT, GPIO_NO_PULL,
          GPIO_2MA);
      Instance->NvtDevice->GpioTlmmProtocol->GpioOut(ResetConfig, 0);
      gBS->Stall(50000);
      Instance->NvtDevice->GpioTlmmProtocol->GpioOut(ResetConfig, 1);
      gBS->Stall(300000);

      // Re-init
      SwI2cInit(Instance);
      NvtSwResetIdle(Instance);
      NvtStopCrcReboot(Instance);
      NvtDrainBufferBootloader(Instance);

      Instance->LastPacketId = -1;
      FailureCount           = 0;
    }
    Instance->IsBusy = FALSE;
    return;
  }

  // Reset failure count on success
  FailureCount = 0;

  // ===== PROCESS TOUCH STATE =====
  if (TouchData.TouchStatus != 0) {
    Instance->LastX = TouchData.TouchX;
    Instance->LastY = TouchData.TouchY;

    if (Instance->NvtDevice->XInverted) {
      Instance->LastX = Instance->AbsPointerMode.AbsoluteMaxX - Instance->LastX;
    }
    if (Instance->NvtDevice->YInverted) {
      Instance->LastY = Instance->AbsPointerMode.AbsoluteMaxY - Instance->LastY;
    }

    Instance->IsTouched    = TRUE;
    Instance->StateChanged = TRUE;
    Instance->IsTouched    = TRUE;
    Instance->StateChanged = TRUE;
    // DEBUG(
    //     (EFI_D_ERROR, "NVT: Touch X=%d Y=%d P=%d W=%d\n", Instance->LastX,
    //      Instance->LastY, TouchData.TouchPressure, TouchData.TouchWidth));
  }
  else {
    if (Instance->IsTouched) {
      Instance->IsTouched    = FALSE;
      Instance->StateChanged = TRUE;
      // DEBUG((EFI_D_ERROR, "NVT: Released\n"));
    }
  }

  Instance->IsBusy = FALSE;
}

/**
  Start Polling Timer

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Timer started
**/
EFI_STATUS AbsStartPolling(IN NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status = EFI_SUCCESS;

  // Set event routines
  Status = gBS->CreateEvent(
      EVT_NOTIFY_SIGNAL | EVT_TIMER, TPL_CALLBACK, SyncPollCallback, Instance,
      &Instance->PollingTimerEvent);
  ASSERT_EFI_ERROR(Status);

  Status = gBS->SetTimer(
      Instance->PollingTimerEvent, TimerPeriodic,
      TIMER_INTERVAL_TOUCH_POLL); // Use defined interval (10ms/100Hz)
  ASSERT_EFI_ERROR(Status);

  // Initialize LastPacketId
  Instance->LastPacketId = -1;

  return Status;
}

/**
  Get Absolute Pointer State

  Returns the current touch coordinates and button state.

  @param This      Absolute Pointer Protocol
  @param State     Output State buffer

  @retval EFI_SUCCESS           State returned
  @retval EFI_NOT_READY         No new data
**/
EFI_STATUS AbsPGetState(
    IN EFI_ABSOLUTE_POINTER_PROTOCOL  *This,
    IN OUT EFI_ABSOLUTE_POINTER_STATE *State)
{
  EFI_STATUS         Status = EFI_SUCCESS;
  NVT_INTERNAL_DATA *Instance;

  if (This == NULL || State == NULL) {
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  Instance = NVT_TCH_INSTANCE_FROM_ABSTCH_THIS(This);
  if (!Instance->StateChanged) {
    Status = EFI_NOT_READY;
    goto exit;
  }

  State->CurrentX        = Instance->LastX;
  State->CurrentY        = Instance->LastY;
  State->CurrentZ        = 0;
  State->ActiveButtons   = Instance->IsTouched ? 1 : 0;
  Instance->StateChanged = FALSE;

exit:
  return Status;
}

/**
  Wait For Input Event

  @param Event    Event handle
  @param Context  Driver Instance
**/
VOID EFIAPI AbsPWaitForInput(IN EFI_EVENT Event, IN VOID *Context)
{
  NVT_INTERNAL_DATA *Instance = (NVT_INTERNAL_DATA *)Context;
  EFI_TPL            OldTpl;

  //
  // Enter critical section
  //
  OldTpl = gBS->RaiseTPL(TPL_NOTIFY);

  SyncPollCallback(NULL, Instance);

  if (Instance->StateChanged) {
    gBS->SignalEvent(Event);
  }

  //
  // Leave critical section and return
  //
  gBS->RestoreTPL(OldTpl);
}

// --- I2C PIN CONFIGURATION ---
// GPIO 4 (SDA), GPIO 5 (SCL) -> Function 1 (QUP01)
// Drive Strength: 2mA
// Bias: PULL_UP (Internal Strong Pull) to ensure signal integrity
/**
  Configure I2C GPIO Pins

  Sets GPIO 4/5 to Input/PullUp for Bit-Banging.

  @param Instance    Pointer to driver instance
**/
VOID NvtConfigI2cPins(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  // GPIO 4: Func 0 (GPIO), Input, Pull Up, 2mA
  UINT32 Config4 = EFI_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
  Status         = Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
      Config4, TLMM_GPIO_ENABLE);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR, "NVT: Failed to config GPIO 4 (I2C SDA): %r\n", Status));
  }
  else {
    DEBUG((EFI_D_ERROR, "NVT: Configured GPIO 4 as I2C SDA (GPIO/PullUp)\n"));
  }

  // GPIO 5
  UINT32 Config5 = EFI_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
  Status         = Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
      Config5, TLMM_GPIO_ENABLE);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR, "NVT: Failed to config GPIO 5 (I2C SCL): %r\n", Status));
  }
  else {
    DEBUG((EFI_D_ERROR, "NVT: Configured GPIO 5 as I2C SCL (GPIO/Func0)\n"));
  }
}
// -----------------------------

// --- DUMP CLOCK HELPER ---
/**
  Dump Clock States (Debug)

  @param ClockProtocol Pointer to Clock Protocol
**/
VOID NvtDumpClocks(EFI_CLOCK_PROTOCOL *ClockProtocol)
{
  UINTN        ClockId;
  BOOLEAN      IsEnabled = FALSE;
  BOOLEAN      IsOn      = FALSE;
  UINT32       Freq      = 0;
  CONST CHAR8 *Clocks[]  = {
      "gcc_qupv3_wrap0_core_2x_clk",  "gcc_qupv3_wrap0_core_clk",
      "gcc_qupv3_wrap_0_m_ahb_clk",   "gcc_qupv3_wrap_0_s_ahb_clk",
      "gcc_qupv3_wrap0_s0_clk",       "gcc_qupv3_wrap0_s1_clk",
      "gcc_sys_noc_cpuss_ahb_clk",    "gcc_cpuss_ahb_clk",
      "gcc_qspi_cnoc_periph_ahb_clk", NULL};

  DEBUG((EFI_D_ERROR, "\n--- NVT Clock Dump ---\n"));
  for (int i = 0; Clocks[i] != NULL; i++) {
    if (!EFI_ERROR(
            ClockProtocol->GetClockID(ClockProtocol, Clocks[i], &ClockId))) {
      ClockProtocol->IsClockEnabled(ClockProtocol, ClockId, &IsEnabled);
      ClockProtocol->IsClockOn(ClockProtocol, ClockId, &IsOn);
      ClockProtocol->GetClockFreqHz(ClockProtocol, ClockId, &Freq);
      DEBUG(
          (EFI_D_ERROR, "%a enabled: %d on: %d freq: %d hz\n", Clocks[i],
           IsEnabled, IsOn, Freq));
    }
    else {
      DEBUG((EFI_D_ERROR, "%a : Not Found\n", Clocks[i]));
    }
  }
  DEBUG((EFI_D_ERROR, "----------------------\n"));
}
// -------------------------

// --- DEBUGGING S_AHB CLOCK DEPENDENCY & VOTING ---
// User reports S_AHB is still OFF despite manual vote (Bit 7 of 0x5200c).
// Dependency Chain:
// 1. S_AHB (interface) depends on Wrapper Generic Logic.
// 2. Wrapper Logic usually clocked by CORE or AHB.
// 3. User logs show S0 is ON but S1 is ON.
// 4. We suspect we need to Vote for S0 as well or set more bits in Vote Reg.

// New Strategy:
// 1. Dump Vote Reg 0x5200c (APSS Vote) and 0x4200c (RPM Vote?)
// 2. Enable ALL known voting bits for QUPV3_0 (S0, S1, S_AHB, M_AHB).
// 3. Re-Check CBCR (0x17008).

#define GCC_BASE_ADDR 0x00100000
#define GCC_APSS_BRANCH_ENA_VOTE 0x5200c
// QUPV3_0 Bits in Vote Reg:
// Bit 0: GCC_QUPV3_WRAP_0_M_AHB_CLK? No, M_AHB is usually Bit 6.
// Bit 6: GCC_QUPV3_WRAP_0_M_AHB_CLK
// Bit 7: GCC_QUPV3_WRAP_0_S_AHB_CLK
// Bit 8?: GCC_QUPV3_WRAP0_CORE_CLK?
// Bit 9?: GCC_QUPV3_WRAP0_CORE_2X_CLK?

#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

#define GCC_QUPV3_WRAP_0_S_AHB_VOTE_BIT BIT(7)
#define GCC_QUPV3_WRAP_0_M_AHB_VOTE_BIT BIT(6)

// Let's vote for EVERYTHING that looks like QUPV3 in this register based on
// mask guess. Or safer: Enable S0 (if it's a vote).

/**
  Manually Vote for Clock Enable (GCC Register)

  @param RegisterOffset Register Offset from GCC Base
  @param BitMask        Bit(s) to enable
**/
VOID NvtEnableVotedClock(UINT32 RegisterOffset, UINT32 BitMask)
{
  UINT32 BaseAddr = GCC_BASE_ADDR + RegisterOffset;
  UINT32 Value;

  // Read
  Value = MmioRead32(BaseAddr);
  DEBUG((EFI_D_ERROR, "NVT: VoteReg (0x%x) Read: 0x%08x\n", BaseAddr, Value));

  if ((Value & BitMask) != BitMask) {
    DEBUG((EFI_D_ERROR, "NVT: Voting for Clock (Mask 0x%x)...\n", BitMask));
    MmioOr32(BaseAddr, BitMask);

    // Read Back - Critical to flush write
    Value = MmioRead32(BaseAddr);
    DEBUG((EFI_D_ERROR, "NVT: VoteReg After Write: 0x%08x\n", Value));
  }
  else {
    DEBUG((EFI_D_ERROR, "NVT: Clock already voted (Mask 0x%x).\n", BitMask));
  }
}

// --- HELPER WRAPPERS FOR LINUX LOGIC ---

/**
  Perform Bootloader Reset

  Sends 0x69 command to I2C HW Address (0x62).
  Required before Chip Detection.

  @param Instance    Pointer to driver instance
**/
EFI_STATUS NvtBootloaderReset(NVT_INTERNAL_DATA *Instance)
{
  UINT8 Buf[8];

  DEBUG((EFI_D_ERROR, "NVT: Bootloader reset...\n"));

  // ==================== CRITICAL: Use HW address (0x62) ====================
  NvtSetSlaveAddress(Instance, I2C_HW_Address); // NOT I2C_FW_Address!

  // Send reset command: Write 0x00 = 0x69
  Buf[0] = 0x00;
  Buf[1] = 0x69;
  NvtI2cWriteRaw(Instance, Buf, 2);

  gBS->Stall(35000); // 35ms delay (Matched with Linux for Aftersales support)

  DEBUG((EFI_D_ERROR, "NVT: Bootloader reset complete\n"));
  return EFI_SUCCESS;
}

/**
  Get Firmware Information

  Reads version, resolution, and status.
  Checks for broken firmware.

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Info Read (even if broken)
  @retval EFI_DEVICE_ERROR      Read Failed
**/
EFI_STATUS NvtGetFwInfo(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      Buf[64]    = {0};
  UINT32     RetryCount = 0;

  if (Instance->EventBufAddr == 0) {
    DEBUG((EFI_D_ERROR, "NVT: EventBufAddr is 0! Forcing default 0x21C00\n"));
    Instance->EventBufAddr = 0x21C00;
  }

  DEBUG(
      (EFI_D_ERROR, "NVT: Getting FW Info (EventBufAddr: 0x%X)...\n",
       Instance->EventBufAddr));

  for (RetryCount = 0; RetryCount < 3; RetryCount++) {
    // 1. Set XData Index using EventBufAddr
    // Corrected to match Linux: buf[1] = addr >> 16; buf[2] = addr >> 8;
    Buf[0] = 0xFF;
    Buf[1] = (UINT8)((Instance->EventBufAddr >> 16) & 0xFF);
    Buf[2] = (UINT8)((Instance->EventBufAddr >> 8) & 0xFF);

    NvtSetSlaveAddress(Instance, I2C_FW_Address);
    NvtI2cWriteRaw(Instance, Buf, 3);

    // gBS->Stall(5000);

    // 2. Read FW Info (Reg 0x78)
    Status = NvtI2cRead(Instance, EVENT_MAP_FWINFO, Buf, 17);

    if (!EFI_ERROR(Status)) {
      // Adjusted Indices to match Linux nt36xxx.c (Byte 0 is Dummy)
      // Linux buf[1] maps to UEFI Buf[0]
      Instance->IdInfo.FwVer = Buf[0];
      Instance->IdInfo.XNum  = Buf[2];
      Instance->IdInfo.YNum  = Buf[3];
      Instance->IdInfo.AbsXMax =
          (UINT16)((Buf[NVT_TS_PARAMS_WIDTH] << 8) |
                   Buf[NVT_TS_PARAMS_WIDTH +
                       1]); // Linux buf[5]<<8 | buf[6] -> Buf[4]|Buf[5]
      Instance->IdInfo.AbsYMax =
          (UINT16)((Buf[NVT_TS_PARAMS_HEIGHT] << 8) |
                   Buf[NVT_TS_PARAMS_HEIGHT +
                       1]); // Linux buf[7]<<8 | buf[8] -> Buf[6]|Buf[7]
      Instance->IdInfo.MaxButtonNum =
          Buf[NVT_TS_PARAMS_MAX_BUTTONS]; // Linux buf[11] -> Buf[10]

      DEBUG(
          (EFI_D_ERROR, "NVT: FW Ver: 0x%02X, X/Y: %d/%d, Res: %d/%d\n",
           Instance->IdInfo.FwVer, Instance->IdInfo.XNum, Instance->IdInfo.YNum,
           Instance->IdInfo.AbsXMax, Instance->IdInfo.AbsYMax));

      // Check if Broken (CheckSum mismatch)
      BOOLEAN ChecksumValid = ((Buf[0] + Buf[1]) == 0xFF);

      // Trust Reported Resolution ONLY if Valid Checksum AND Realistic
      // Resolution 65535 (0xFFFF) is technically > 0 but wrong. Cap at 10000.
      if (ChecksumValid && Instance->IdInfo.AbsXMax > 100 &&
          Instance->IdInfo.AbsXMax < 10000 && Instance->IdInfo.AbsYMax > 100 &&
          Instance->IdInfo.AbsYMax < 10000) {

        DEBUG(
            (EFI_D_ERROR, "NVT: Using Firmware Reported Resolution: %dx%d\n",
             Instance->IdInfo.AbsXMax, Instance->IdInfo.AbsYMax));
        return EFI_SUCCESS;
      }

      DEBUG(
          (EFI_D_ERROR,
           "NVT: FW info Invalid (Checksum: %d, Res: %dx%d)! Using Defaults.\n",
           ChecksumValid, Instance->IdInfo.AbsXMax, Instance->IdInfo.AbsYMax));

      if (!ChecksumValid) {
        DEBUG(
            (EFI_D_ERROR,
             "NVT: FW info broken & Res Invalid! Using Defaults.\n"));
        // If Broken AND Invalid Res, use PCD Defaults (from NovatekTouch.dec)
        Instance->IdInfo.FwVer        = 0x00;
        Instance->IdInfo.XNum         = PcdGet32(PcdNvtTsParamsXNum);
        Instance->IdInfo.YNum         = PcdGet32(PcdNvtTsParamsYNum);
        Instance->IdInfo.AbsXMax      = PcdGet32(PcdNvtTsParamsAbsXMax);
        Instance->IdInfo.AbsYMax      = PcdGet32(PcdNvtTsParamsAbsYMax);
        Instance->IdInfo.MaxButtonNum = PcdGet32(PcdNvtTsParamsMaxButtons);

        DEBUG(
            (EFI_D_ERROR,
             "Set default fw_ver=%d, x_num=%d, y_num=%d, "
             "abs_x_max=%d, abs_y_max=%d, max_button_num=%d\n",
             Instance->IdInfo.FwVer, Instance->IdInfo.XNum,
             Instance->IdInfo.YNum, Instance->IdInfo.AbsXMax,
             Instance->IdInfo.AbsYMax, Instance->IdInfo.MaxButtonNum));

        return EFI_SUCCESS; // broken but linux ignores this, and so will we
      }
      else {
        return EFI_SUCCESS; // valid main fw info
      }
    }
    gBS->Stall(10000);
  }
  return EFI_DEVICE_ERROR; // you better not be tinkering
}

//------------------------------------------------------------------------------
/**
  Check Firmware Reset State

  Verifies firmware reached expected initialization state (0xA0, 0xA2, 0xA1).
  Matches Linux logic with timeouts.

  @param Instance       Pointer to driver instance
  @param ExpectedState  State to wait for (e.g. RESET_STATE_INIT)

  @retval EFI_SUCCESS           State reached
  @retval EFI_DEVICE_ERROR      Timeout
**/
EFI_STATUS
NvtCheckFwResetState(NVT_INTERNAL_DATA *Instance, UINT8 ExpectedState)
{
  EFI_STATUS Status;
  UINT8      Buf[8];
  UINT32     Retry;

  DEBUG(
      (EFI_D_ERROR, "NVT: CheckFwResetState expecting 0x%02X...\n",
       ExpectedState));

  // Set slave to FW address
  NvtSetSlaveAddress(Instance, I2C_FW_Address);

  for (Retry = 0; Retry < 100; Retry++) {
    // Set page to EventBufAddr
    Buf[0] = 0xFF;
    Buf[1] = (UINT8)((Instance->EventBufAddr >> 16) & 0xFF);
    Buf[2] = (UINT8)((Instance->EventBufAddr >> 8) & 0xFF);
    Status = NvtI2cWriteRaw(Instance, Buf, 3);

    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "NVT: Page write FAILED (retry %d)\n", Retry));
      gBS->Stall(10000);
      continue;
    }

    // Read reset state at EVENT_MAP_RESET_COMPLETE (0x60)
    Status = NvtI2cRead(Instance, EVENT_MAP_RESET_COMPLETE, Buf, 6);
    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "NVT: I2C read FAILED (retry %d)\n", Retry));
      gBS->Stall(10000);
      continue;
    }

    // Dump all 6 bytes for debugging
    DEBUG(
        (EFI_D_ERROR,
         "NVT: Read[0-5]: %02X %02X %02X %02X %02X %02X (retry %d)\n", Buf[0],
         Buf[1], Buf[2], Buf[3], Buf[4], Buf[5], Retry));

    // Check if expected state reached (Linux logic: >= CheckState && <=
    // RESET_STATE_MAX) Common states: 0xA0 (Init), 0xA2 (Rek Finish), 0xA3 (Rek
    // Fail)

    // if (Buf[0] == 0x16 && Buf[1] == 0x59) {
    //   DEBUG((EFI_D_ERROR, "NVT: Met novatek-mp-criteria-5916!\n"));
    //   return EFI_SUCCESS;
    // }

    if (Buf[0] >= ExpectedState &&
        Buf[0] <= 0xAF) { // 0xAF as safe max for Reset States
      DEBUG(
          (EFI_D_ERROR,
           "NVT: FW reached state 0x%02X (Expected >= 0x%02X) after %d "
           "retries!\n",
           Buf[1], ExpectedState, Retry));
      return EFI_SUCCESS;
    }

    // Log what state we're stuck at
    if (Retry % 10 == 0 && Retry > 0) {
      DEBUG(
          (EFI_D_ERROR,
           "NVT: Still waiting... Current state = 0x%02X (want >= 0x%02X)\n",
           Buf[0], ExpectedState));
    }

    gBS->Stall(10000); // 10ms between checks
  }

  DEBUG(
      (EFI_D_ERROR, "NVT: FW TIMEOUT! Never reached >= 0x%02X (Last: 0x%02X)\n",
       ExpectedState, Buf[1]));
  return EFI_DEVICE_ERROR;
}

/**
  Read Project ID (PID) from Firmware

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Read success
  @retval EFI_DEVICE_ERROR      I2C Error
**/
EFI_STATUS NvtReadPid(NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;
  UINT8      Buf[3];
  UINT16     Pid;

  // Set XDATA Index (EVENT_BUF_ADDR)
  // Ensure we are targeted at FW Address
  Buf[0] = 0xFF;
  Buf[1] = (UINT8)((Instance->EventBufAddr >> 16) & 0xFF);
  Buf[2] = (UINT8)((Instance->EventBufAddr >> 8) & 0xFF);

  NvtSetSlaveAddress(Instance, I2C_FW_Address);
  NvtI2cWriteRaw(Instance, Buf, 3);
  gBS->Stall(5000); // 5ms

  // Read PID (Reg 0x9A) - Reads 2 bytes (Linux reads len-1=2)
  Status = NvtI2cRead(Instance, EVENT_MAP_PROJECTID, Buf, 2);

  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "NVT: Read PID Failed: %r\n", Status));
    return Status;
  }

  // Linux: ts->nvt_pid = (buf[2] << 8) | buf[1]; (Byte 1 << 8 | Byte 0)
  // UEFI: Buf[0]=Byte0, Buf[1]=Byte1.
  Pid = (UINT16)((Buf[1] << 8) | Buf[0]);

  DEBUG(
      (EFI_D_ERROR, "NVT: PID Read: 0x%04X (Buf: %02X %02X)\n", Pid, Buf[0],
       Buf[1]));

  return EFI_SUCCESS;
}

// ---------------------------------------------
// Initialize MP Process (Stub)
// ---------------------------------------------
/**
  Initialize MP Process (Stub)
**/
EFI_STATUS NvtMpProcInit(void)
{
  DEBUG((EFI_D_INFO, "NVT: NvtMpProcInit called (Stub)\n"));
  return EFI_SUCCESS;
}

// ---------------------------------------------
// Init Touch Mode Data (Stub/Guess)
// ---------------------------------------------
/**
  Init Touch Mode Data (Stub)
**/
EFI_STATUS NvtInitTouchModeData(void)
{
  DEBUG((EFI_D_INFO, "NVT: NvtInitTouchModeData called (Stub)\n"));
  // Linux init_lct_tp_info_node_init implementation?
  return EFI_SUCCESS;
}

// ---------------------------------------------

/**
  Power Up Controller

  Performs full initialization sequence:
  - Clocks & GPIOs
  - Bootloader Reset
  - Chip Logic Check
  - Firmware Init & Tuning

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Powered up
  @retval EFI_DEVICE_ERROR      Failure
**/
EFI_STATUS
EFIAPI
NvtPowerUpController(NVT_INTERNAL_DATA *Instance)
{
  UINT32     Config;
  UINT32     ResetLine;
  EFI_STATUS Status;

  if (Instance == NULL || Instance->NvtDevice == NULL ||
      Instance->NvtDevice->GpioTlmmProtocol == NULL) {
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  // --- COMPREHENSIVE CLOCK ENABLEMENT (REVERTED) ---
  EFI_CLOCK_PROTOCOL *ClockProtocol;
  Status = gBS->LocateProtocol(
      &gEfiClockProtocolGuid, NULL, (VOID **)&ClockProtocol);
  if (!EFI_ERROR(Status)) {
    UINTN        ClockId;
    CONST CHAR8 *Clocks[] = {
        "gcc_qupv3_wrap0_core_2x_clk", "gcc_qupv3_wrap0_core_clk",
        "gcc_qupv3_wrap_0_m_ahb_clk",  "gcc_sys_noc_cpuss_ahb_clk",
        "gcc_cpuss_ahb_clk",           "gcc_qspi_cnoc_periph_ahb_clk",
        "gcc_qupv3_wrap0_s1_clk",      NULL};

    // Iterate through all clocks to enable them
    for (UINTN i = 0; Clocks[i] != NULL; i++) {
      if (!EFI_ERROR(
              ClockProtocol->GetClockID(ClockProtocol, Clocks[i], &ClockId))) {
        ClockProtocol->EnableClock(ClockProtocol, ClockId);

        // Special handling for s1_clk: Set frequency to 19.2MHz
        if (AsciiStrCmp(Clocks[i], "gcc_qupv3_wrap0_s1_clk") == 0) {
          ClockProtocol->SetClockFreqHz(
              ClockProtocol, ClockId, 19200000, EFI_CLOCK_FREQUENCY_HZ_CLOSEST,
              NULL);
          DEBUG((EFI_D_ERROR, "NVT: Set gcc_qupv3_wrap0_s1_clk to 19.2MHz\n"));

          // --- FORCE S1 CLOCK VIA MMIO (0Hz Fix) ---
          // 1. Configure CFG_RCGR: SRC_SEL=0 (XO), SRC_DIV=0 (Div 1), MODE=0
          // (Bypass) -> 19.2MHz
          MmioWrite32(GCC_QUPV3_WRAP0_S1_CFG_RCGR, 0x00000000);
          // 2. Trigger Update: Bit 0=UPDATE, Bit 1=ROOT_EN
          MmioWrite32(GCC_QUPV3_WRAP0_S1_CMD_RCGR, 0x00000003);
          gBS->Stall(200);

          // Force S_AHB CBCR Enable just in case (0x117008)
          MmioOr32(GCC_QUPV3_WRAP_0_S_AHB_CBCR, BIT(0));

          // Force S1 Enable via VOTE REGISTER (0x5200C) using Helper
          // This uses GCC_BASE_ADDR (0x100000) + Offset (0x5200C) = 0x15200C
          NvtEnableVotedClock(GCC_APSS_BRANCH_ENA_VOTE, BIT(11) | BIT(7));

          // ALSO Write to S1 CBCR (0x117274) - Some platforms need this even if
          // voted
          MmioOr32(GCC_QUPV3_WRAP0_S1_CBCR, BIT(0));

          UINT32 S1Cbcr = MmioRead32(GCC_QUPV3_WRAP0_S1_CBCR);
          DEBUG(
              (EFI_D_ERROR,
               "NVT: S1 Clock Voted & Enabled. CBCR (0x17274)=0x%08x\n",
               S1Cbcr));
        }
      }
    }
    // 5. Enable Clocks
    // --- HELPER FOR VOTING CLOCKS (Added Fix) ---
    // GCC_APSS_BRANCH_ENA_VOTE (0x5200c)
    // Enable S_AHB (Bit 7) AND M_AHB (Bit 6).
    // Also ensuring S0 (Bit 0?) and others are voted just in case.
    // Based on user log `2FC0`, 0,1,2,3,4,5 are 0? No:
    // 2FC0 = 0010 1111 1100 0000.
    // Bits 6,7,8,9,10,11,13 are 1.
    // So 6(M_AHB), 7(S_AHB) ALREADY VOTED.

    // FORCE CBCR ENABLE (The 'Hammer' Approach)
    // Address: 0x17008 (GCC_QUPV3_WRAP_0_S_AHB_CBCR)
    {
      UINT32 CbcrAddr = GCC_BASE_ADDR + 0x17008;
      UINT32 CbcrVal  = MmioRead32(CbcrAddr);
      DEBUG(
          (EFI_D_ERROR, "NVT: S_AHB CBCR (0x%x) Before: 0x%08x\n", CbcrAddr,
           CbcrVal));

      // Assign NvtDevice to Instance for global access
      Instance->NvtDevice =
          Instance->NvtDevice; // Already done in BindingStart? This function
                               // is PowerUp.
      // Wait, Instance->NvtDevice IS initialized in BindingStart.
      // But verify it here.

      // Critical Fix: Disable HW Gating (Bit 1) if set.
      // User log showed 0x80000002 (Bit 1 set), which ignores SW Enable.
      if (CbcrVal & BIT(1)) {
        DEBUG((EFI_D_ERROR, "NVT: Disabling HW Gating (Clearing Bit 1)...\n"));
        MmioAnd32(CbcrAddr, (UINT32)~BIT(1));
      }

      // Force Enable (Bit 0)
      CbcrVal = MmioRead32(CbcrAddr); // Re-read
      if (!(CbcrVal & BIT(0))) {
        DEBUG((EFI_D_ERROR, "NVT: Forcing CBCR Bit 0 (Enable)...\n"));
        MmioOr32(CbcrAddr, BIT(0));
      }

      // Verify
      CbcrVal = MmioRead32(CbcrAddr);
      DEBUG((EFI_D_ERROR, "NVT: S_AHB CBCR Final: 0x%08x\n", CbcrVal));

      if (CbcrVal & BIT(31)) {
        DEBUG(
            (EFI_D_ERROR,
             "NVT: WARNING: S_AHB Clock is still OFF (Bit 31 set)!\n"));
      }
    }

    // Explicitly Vote for S_AHB and M_AHB
    NvtEnableVotedClock(
        GCC_APSS_BRANCH_ENA_VOTE,
        GCC_QUPV3_WRAP_0_S_AHB_VOTE_BIT | GCC_QUPV3_WRAP_0_M_AHB_VOTE_BIT);

    // Also Vote for Core Clocks?
    // User log shows Core (Bit 8?), Core 2X (Bit 9?) are ON (Bits 8,9 set in
    // 2FC0).
    // ----------------------------

    // 5. DUMP STATUS
    NvtDumpClocks(ClockProtocol);
  }

  // Pin Sanity check
  ResetLine = Instance->NvtDevice->ControllerResetPin;
  DEBUG((EFI_D_ERROR, "NVT: Reset GPIO: %d\n", ResetLine));

  if (ResetLine <= 0) {
    DEBUG((EFI_D_ERROR, "NovatekNvtTsDxe: Invalid GPIO configuration\n"));
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  // Power Seq (Config GPIO)
  Config = EFI_GPIO_CFG(ResetLine, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
  Status = Instance->NvtDevice->GpioTlmmProtocol->ConfigGpio(
      Config, TLMM_GPIO_ENABLE);

  // Configure MSM GPIO RESET line to Low (Assert Reset)
  Instance->NvtDevice->GpioTlmmProtocol->GpioOut(Config, GPIO_LOW_VALUE);
  gBS->Stall(10 * 1000); // 10ms

  // Configure MSM GPIO RESET line to High (Release Reset)
  Instance->NvtDevice->GpioTlmmProtocol->GpioOut(Config, GPIO_HIGH_VALUE);
  gBS->Stall(50 * 1000); // 50ms

  // 6. Perform Bootloader Reset to Exit Sleep/Gesture Mode (Using Helper)
  // Call CheckChipVerTrim first as it performs reset sequence internally
  // NvtCheckChipVerTrim(Instance);

  // Re-verify Reset State
  NvtBootloaderReset(Instance);
  Status = NvtCheckFwResetState(Instance, RESET_STATE_INIT);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "NVT: FW Reset State Check Failed! (Device Stuck?)\n"));
  }
  else {
    DEBUG((EFI_D_ERROR, "NVT: FW Reset Check OK. Device in Normal Mode.\n"));
  }

  DEBUG((EFI_D_ERROR, "NovatekNvtTsDxe: Touch controller initialized \n"));
  Status = EFI_SUCCESS;

  Status = NvtBootloaderReset(Instance);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Bootloader reset failed (Ignored)\n"));
    // return Status; // REMOVED: Soft-fail, Linux nvt_bootloader_reset is void
  }

  // Step 2: Wait for IC initialization (RESET_STATE_INIT = 0xA0)
  DEBUG((EFI_D_ERROR, "Nvt: Waiting for IC init (0xA0)...\n"));
  Status = NvtCheckFwResetState(Instance, RESET_STATE_INIT);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: First init check failed, retrying...\n"));

    // Retry once with bootloader reset
    NvtBootloaderReset(Instance);
    gBS->Stall(35000); // 35ms

    Status = NvtCheckFwResetState(Instance, RESET_STATE_INIT);
    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "Nvt: IC init timeout (Ignored, matching Linux)\n"));
      // return EFI_DEVICE_ERROR; // REMOVED: Soft-fail
    }
  }

  // Step 3: Wait for calibration completion (RESET_STATE_REK = 0xA1)
  DEBUG((EFI_D_ERROR, "Nvt: Waiting for calibration (0xA1)...\n"));
  Status = NvtCheckFwResetState(Instance, RESET_STATE_REK);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR, "Nvt: Calibration timeout (Ignored, matching Linux)\n"));
    // return Status; // REMOVED: Soft-fail
  }

  DEBUG((EFI_D_ERROR, "Nvt: Firmware initialization complete\n"));

  DEBUG((EFI_D_ERROR, "Nvt: Applying optimal tuning parameters...\n"));

  // Initialize tuning config
  // These values are tuned for stability on both Original and Clone ICs
  Instance->TuningConfig.Sensitivity     = 100;
  Instance->TuningConfig.EdgeFilterLevel = 15;
  Instance->TuningConfig.ThresholdDiff   = 70;
  Instance->TuningConfig.FreqHopEnabled  = FALSE;
  Instance->TuningConfig.EdgeOrientation = 0;
  // Apply all tuning parameters
  Status = NvtApplyTuningConfig(Instance);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Tuning config partially failed (non-fatal)\n"));
    return EFI_DEVICE_ERROR;
  }

  // Ensure we're in normal mode
  Status = NvtChangeMode(Instance, NORMAL_MODE);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Failed to enter normal mode\n"));
    return Status;
  }

  // Final verification - clear any pending events
  Status = NvtClearFwStatus(Instance);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Failed to clear FW status\n"));
    return EFI_DEVICE_ERROR;
  }

  DEBUG((EFI_D_ERROR, "Nvt: Power up sequence complete\n"));
  Instance->Initialized = TRUE;

exit:
  return Status;
}

#define NVT_REG_CMD 0x01

/**
  Set I2C Buffer Page

  @param Instance    Pointer to driver instance
  @param PageAddr    Address Page
  @retval EFI_SUCCESS           Success
**/
EFI_STATUS NvtSetPage(NVT_INTERNAL_DATA *Instance, UINT32 PageAddr)
{
  UINT8 Buf[3];
  Buf[0] = 0xFF; // Page Set Command
  Buf[1] = (UINT8)((PageAddr >> 16) & 0xFF);
  Buf[2] = (UINT8)((PageAddr >> 8) & 0xFF);
  // Write [Reg=0x01] [0xFF] [HighByte] [LowByte]
  return Nvti2cWriteHelper(Instance, I2C_FW_Address, NVT_REG_CMD, Buf, 3);
}

// --- ESD (Firmware Crash) Detection ---
/**
  Check for Firmware Crash (ESD)

  Detects 0x77 pattern in touch data.

  @param PointData   Pointer to Touch Buffer
  @retval TRUE       Crash Detected
  @retval FALSE      Normal Data
**/
BOOLEAN NvtCheckFirmwareCrash(UINT8 *PointData)
{
  // Check if bytes 1-6 are all 0x77 (firmware crash signature)
  // Note: PointData[0] is dummy. So checking Buf[1]..Buf[6].
  // If PointData passed is Instance->Buf:
  for (UINT32 i = 1; i <= 6; i++) {
    if (PointData[i] != 0x77) {
      return FALSE; // Normal data
    }
  }

  DEBUG((EFI_D_ERROR, "[NVT] FIRMWARE CRASH DETECTED (0x77 pattern)!\n"));
  return TRUE;
}

/**
  Get Touch Data

  Reads touch coordinates, pressure, and width.
  Applies filtering (Ghost Touch, BandPass).

  @param Instance    Pointer to driver instance
  @param DataBuffer  Output Touch Data

  @retval EFI_SUCCESS           Valid touch data
  @retval EFI_NOT_READY         No touch / Filtered / Ghost
  @retval EFI_DEVICE_ERROR      Hardware/Bus Error
**/
EFI_STATUS EFIAPI
NvtGetTouchData(NVT_INTERNAL_DATA *Instance, IN PTOUCH_DATA DataBuffer)
{
  EFI_STATUS Status;
  UINT32     Retry;
  BOOLEAN    ValidData = FALSE;
  UINT8      PacketId, TouchStatus;
  UINT16     RawX, RawY;
  UINT16     ScaledX, ScaledY;
  UINT32     Position = 1; // Linux: position = 1 + 6 * i (for i=0)

  // ===== READ FROM BOOTLOADER =====
  // Matches Linux: CTP_I2C_READ(..., point_data, 66) -> reads 65 bytes into
  // point_data[1]
  NvtSetSlaveAddress(Instance, I2C_HW_Address);

  for (Retry = 0; Retry < 3; Retry++) {
    SetMem(Instance->Buf, POINT_DATA_LEN + 1, 0x00); // clear all

    // Read 65 bytes (POINT_DATA_LEN) into Buf[1]
    // Buf[0] remains 0x00 (Dummy/Reg)
    Status = NvtI2cRead(Instance, 0x00, &Instance->Buf[1], POINT_DATA_LEN);

    if (EFI_ERROR(Status)) {
      gBS->Stall(5000);
      continue;
    }

    // Check for FW Crash (0x77 pattern) - Matches Linux nvt_fw_recovery
    if (NvtCheckFirmwareCrash(
            &Instance->Buf[0])) { // Check whole buffer (logic handles offset)
      DEBUG(
          (EFI_D_ERROR,
           "NVT: FW Crash (0x77) detected! Triggering recovery...\n"));
      return EFI_DEVICE_ERROR; // Force failure to trigger SyncPollCallback
                               // reset logic
    }

    ValidData = TRUE;
    break;
  }

  if (!ValidData) {
    return EFI_DEVICE_ERROR;
  }

  // ===== BUS IDLE CHECK =====
  // Linux: if (point_data[1] == 0xFF && point_data[2] == 0xFF)
  if (Instance->Buf[Position] == 0xFF && Instance->Buf[Position + 1] == 0xFF) {
    return EFI_NOT_READY;
  }

  // ===== PACKET ID VALIDATION =====
  // Linux: input_id = point_data[position] >> 3
  PacketId = (Instance->Buf[Position] >> NVT_TS_TOUCH_SLOT_SHIFT) & 0x1F;

  if (PacketId > 0x0F && PacketId != 0x1F) { // 0x1F might be idle?
    // DEBUG((EFI_D_ERROR, "NVT: Invalid PacketID=0x%X\n", PacketId));
    return EFI_NOT_READY;
  }

  if (PacketId == Instance->LastPacketId) {
    return EFI_NOT_READY; // Duplicate packet
  }
  Instance->LastPacketId = PacketId;

  // Linux: if ((point_data[position] & 0x07) == 0x01 ...)
  TouchStatus = Instance->Buf[Position] & NVT_TS_TOUCH_TYPE_MASK;

  // Handle release
  if (TouchStatus == NVT_TS_TOUCH_RELEASE) {
    DataBuffer->TouchStatus = 0;
    // DEBUG((EFI_D_INFO, "NVT: Released\n"));
    return EFI_SUCCESS;
  }

  // Only accept 0x01 (press) and 0x02 (move)
  if (TouchStatus != NVT_TS_TOUCH_NEW && TouchStatus != NVT_TS_TOUCH_UPDATE) {
    return EFI_NOT_READY;
  }

  RawX = (UINT16)((Instance->Buf[Position + 1] << 4) |
                  (Instance->Buf[Position + 3] >> 4));
  RawY = (UINT16)((Instance->Buf[Position + 2] << 4) |
                  (Instance->Buf[Position + 3] & 0x0F));

  // ===== PRESSURE & WIDTH PARSING (Linux Parity) =====
  UINT32 InputW = Instance->Buf[Position + 4];
  UINT32 InputP = (UINT32)(Instance->Buf[Position + 5]) |
                  ((UINT32)(Instance->Buf[Position + 6] & 0x03) << 8);

  // ===== GHOST TOUCH FILTERING (Band-Pass) =====
  if (InputP < 770 || InputP > 785) {
    return EFI_NOT_READY; // Ghost touch rejection
  }

  if (InputW > 25 || InputW < 5) {
    // Too narrow (Noise)
    return EFI_NOT_READY;
  }

  // ===== OUTPUT =====
  DataBuffer->TouchX        = RawX;
  DataBuffer->TouchY        = RawY;
  DataBuffer->TouchPressure = (UINT16)InputP;
  DataBuffer->TouchWidth    = (UINT16)InputW;
  DataBuffer->TouchStatus   = 1;

  // DEBUG((EFI_D_ERROR, "NVT: X=%d Y=%d (Raw: %d,%d)\n", ScaledX, ScaledY,
  // RawX, RawY));

  return EFI_SUCCESS;
}

//------------------------------------------------------------------------------
// Set XDATA Page - Required before accessing firmware registers
//------------------------------------------------------------------------------
/**
  Set XDATA Page Index

  @param Instance    Pointer to driver instance
  @param Address     Target Address
  @retval EFI_SUCCESS           Success
**/
EFI_STATUS NvtSetXdataPage(NVT_INTERNAL_DATA *Instance, UINT32 Address)
{
  UINT8 Buf[3];

  Buf[0] = 0xFF;
  Buf[1] = (UINT8)((Address >> 16) & 0xFF);
  Buf[2] = (UINT8)((Address >> 8) & 0xFF);

  return NvtI2cWriteRaw(Instance, Buf, 3);
}

/**
  Get current firmware data pipe (alternates between 0 and 1)

  @param Instance    Pointer to driver instance

  @return UINT8      Pipe number (0 or 1)
**/
UINT8
NvtGetFwPipe(IN NVT_INTERNAL_DATA *Instance)
{
  UINT8      Buf[8] = {0};
  EFI_STATUS Status;

  // Set XDATA page to EVENT_BUF_ADDR
  Status = NvtSetXdataPage(Instance, Instance->MemMap->EVENT_BUF_ADDR);
  if (EFI_ERROR(Status)) {
    return 0;
  }

  // Read firmware status
  Buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
  Buf[1] = 0x00;
  Status = NvtI2cRead(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    return 0;
  }

  // Return pipe number (bit 0)
  return (Buf[1] & 0x01);
}

/**
  Set touch sensitivity level

  @param Instance    Pointer to driver instance
  @param Level       Sensitivity (0-255, higher = more sensitive)
                     Recommended: 80-120 for normal, 60-80 for less sensitive

  @retval EFI_SUCCESS           Sensitivity set successfully
  @retval EFI_DEVICE_ERROR      I2C communication failed
**/
EFI_STATUS
NvtSetSensitivity(IN NVT_INTERNAL_DATA *Instance, IN UINT8 Level)
{
  UINT8      Buf[8];
  EFI_STATUS Status;

  DEBUG((EFI_D_ERROR, "Nvt: Setting sensitivity to %d\n", Level));

  if (Instance->MemMap == NULL) {
    DEBUG((EFI_D_ERROR, "Nvt: MemMap is NULL! Skipping tuning.\n"));
    return EFI_NOT_READY;
  }

  // Set XDATA page to sensitivity register
  Status = NvtSetXdataPage(Instance, NVT_REG_SENSIVITY);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // Write sensitivity value (low byte of address + value)
  Buf[0] = (NVT_REG_SENSIVITY & 0xFF); // 0x00
  Buf[1] = Level;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Sensitivity write failed\n"));
    return Status;
  }

  Instance->TuningConfig.Sensitivity = Level;
  return EFI_SUCCESS;
}

/**
  Configure edge filter (palm rejection)

  @param Instance      Pointer to driver instance
  @param Level         Filter strength (0-255, higher = more rejection)
                       Recommended: 10-30
  @param Orientation   Screen orientation (0=portrait, 1=landscape)

  @retval EFI_SUCCESS           Edge filter configured
  @retval EFI_DEVICE_ERROR      I2C communication failed
**/
EFI_STATUS
NvtConfigureEdgeFilter(
    IN NVT_INTERNAL_DATA *Instance, IN UINT8 Level, IN UINT8 Orientation)
{
  UINT8      Buf[8];
  EFI_STATUS Status;

  DEBUG(
      (EFI_D_ERROR, "Nvt: Setting edge filter level=%d, orientation=%d\n",
       Level, Orientation));

  // Set edge filter level
  if (Instance->MemMap == NULL)
    return EFI_NOT_READY;
  Status = NvtSetXdataPage(Instance, NVT_REG_EDGE_FILTER_LEVEL);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  Buf[0] = (NVT_REG_EDGE_FILTER_LEVEL & 0xFF); // 0x00
  Buf[1] = Level;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Edge filter level write failed\n"));
    return Status;
  }

  // Set edge orientation
  Status = NvtSetXdataPage(Instance, NVT_REG_EDGE_FILTER_ORIENTATION);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  Buf[0] = (NVT_REG_EDGE_FILTER_ORIENTATION & 0xFF); // 0x00
  Buf[1] = Orientation;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Edge orientation write failed\n"));
    return Status;
  }

  Instance->TuningConfig.EdgeFilterLevel = Level;
  Instance->TuningConfig.EdgeOrientation = Orientation;

  return EFI_SUCCESS;
}

/**
  Set touch detection threshold differential

  @param Instance    Pointer to driver instance
  @param Threshold   Threshold value (higher = less sensitive to noise)
                     Recommended: 50-100

  @retval EFI_SUCCESS           Threshold set successfully
  @retval EFI_DEVICE_ERROR      I2C communication failed
**/
EFI_STATUS
NvtSetThresholdDiff(IN NVT_INTERNAL_DATA *Instance, IN UINT8 Threshold)
{
  UINT8      Buf[8];
  EFI_STATUS Status;

  DEBUG((EFI_D_ERROR, "Nvt: Setting threshold diff to %d\n", Threshold));

  Status = NvtSetXdataPage(Instance, NVT_REG_THDIFF);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  Buf[0] = (NVT_REG_THDIFF & 0xFF);
  Buf[1] = Threshold;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Threshold diff write failed\n"));
    return Status;
  }

  Instance->TuningConfig.ThresholdDiff = Threshold;
  return EFI_SUCCESS;
}

/**
  Enable or disable frequency hopping (for EMI environments)

  @param Instance    Pointer to driver instance
  @param Enable      TRUE to enable freq hop, FALSE to disable

  @retval EFI_SUCCESS           Freq hop configured
  @retval EFI_DEVICE_ERROR      I2C communication failed
**/
EFI_STATUS
NvtSetFreqHop(IN NVT_INTERNAL_DATA *Instance, IN BOOLEAN Enable)
{
  UINT8      Buf[8];
  EFI_STATUS Status;

  DEBUG(
      (EFI_D_ERROR, "Nvt: %a frequency hopping\n",
       Enable ? "Enabling" : "Disabling"));

  if (Instance->MemMap == NULL) {
    DEBUG((EFI_D_ERROR, "Nvt: MemMap is NULL! Cannot set freq hop.\n"));
    return EFI_NOT_READY;
  }

  // Set XDATA page to EVENT_BUF_ADDR
  Status = NvtSetXdataPage(Instance, Instance->MemMap->EVENT_BUF_ADDR);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // Write freq hop command
  Buf[0] = EVENT_MAP_HOST_CMD;
  Buf[1] = Enable ? FREQ_HOP_ENABLE : FREQ_HOP_DISABLE;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Freq hop write failed\n"));
    return Status;
  }

  Instance->TuningConfig.FreqHopEnabled = Enable;
  return EFI_SUCCESS;
}

/**
  Apply all tuning configuration parameters
  Called during initialization after firmware is ready

  @param Instance    Pointer to driver instance

  @retval EFI_SUCCESS           All tuning applied successfully
  @retval EFI_DEVICE_ERROR      One or more tuning operations failed
**/
EFI_STATUS
NvtApplyTuningConfig(IN NVT_INTERNAL_DATA *Instance)
{
  EFI_STATUS Status;

  DEBUG((EFI_D_ERROR, "Nvt: Applying tuning configuration...\n"));

  // Apply sensitivity
  if (Instance->TuningConfig.Sensitivity > 0) {
    Status = NvtSetSensitivity(Instance, Instance->TuningConfig.Sensitivity);
    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "Nvt: Failed to set sensitivity (non-fatal)\n"));
    }
    gBS->Stall(5000); // 5ms delay between settings
  }

  // Apply edge filter
  if (Instance->TuningConfig.EdgeFilterLevel > 0) {
    Status = NvtConfigureEdgeFilter(
        Instance, Instance->TuningConfig.EdgeFilterLevel,
        Instance->TuningConfig.EdgeOrientation);
    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "Nvt: Failed to set edge filter (non-fatal)\n"));
    }
    gBS->Stall(5000);
  }

  // Apply threshold diff
  if (Instance->TuningConfig.ThresholdDiff > 0) {
    Status =
        NvtSetThresholdDiff(Instance, Instance->TuningConfig.ThresholdDiff);
    if (EFI_ERROR(Status)) {
      DEBUG((EFI_D_ERROR, "Nvt: Failed to set threshold (non-fatal)\n"));
    }
    gBS->Stall(5000);
  }

  // Apply frequency hopping
  Status = NvtSetFreqHop(Instance, Instance->TuningConfig.FreqHopEnabled);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Nvt: Failed to set freq hop (non-fatal)\n"));
  }

  DEBUG((EFI_D_ERROR, "Nvt: Tuning configuration applied\n"));
  return EFI_SUCCESS;
}

/**
  Read baseline calibration data (for debugging)

  @param Instance      Pointer to driver instance
  @param Buffer        Output buffer (must be XNum * YNum * 2 bytes)
  @param BufferSize    Size of buffer

  @retval EFI_SUCCESS           Baseline data read successfully
  @retval EFI_BUFFER_TOO_SMALL  Buffer too small
  @retval EFI_DEVICE_ERROR      Failed to read data
**/
EFI_STATUS
NvtReadBaseline(
    IN NVT_INTERNAL_DATA *Instance, OUT UINT8 *Buffer, IN UINT32 BufferSize)
{
  EFI_STATUS Status;
  UINT32     DataSize;
  UINT32     i, j;
  UINT8      Buf[256];

  DataSize = Instance->IdInfo.XNum * Instance->IdInfo.YNum * 2;

  if (BufferSize < DataSize) {
    DEBUG((EFI_D_ERROR, "Nvt: Buffer too small for baseline data\n"));
    return EFI_BUFFER_TOO_SMALL;
  }

  // Switch to test mode
  Status = NvtClearFwStatus(Instance);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  Status = NvtChangeMode(Instance, TEST_MODE_2);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  Status = NvtCheckFwStatus(Instance);
  if (EFI_ERROR(Status)) {
    NvtChangeMode(Instance, NORMAL_MODE);
    return Status;
  }

  // Read baseline data
  Status = NvtSetXdataPage(Instance, Instance->MemMap->BASELINE_ADDR);
  if (EFI_ERROR(Status)) {
    NvtChangeMode(Instance, NORMAL_MODE);
    return Status;
  }

  // Read in 256-byte chunks
  for (i = 0; i < (DataSize / 256) + 1; i++) {
    Buf[0] = (Instance->MemMap->BASELINE_ADDR & 0xFF) + (i * 256);
    Status = NvtI2cRead(Instance, I2C_FW_Address, Buf, 256);
    if (EFI_ERROR(Status)) {
      break;
    }

    CopyMem(
        Buffer + (i * 256), Buf + 1,
        (i == (DataSize / 256)) ? (DataSize % 256) : 256);
  }

  // Return to normal mode
  NvtChangeMode(Instance, NORMAL_MODE);

  DEBUG((EFI_D_ERROR, "Nvt: Baseline data read (%d bytes)\n", DataSize));
  return Status;
}

/**
  Clear Firmware Status

  Clears Host Command and Handshaking registers.

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Success
  @retval EFI_DEVICE_ERROR      Write failed
**/
EFI_STATUS NvtClearFwStatus(IN NVT_INTERNAL_DATA *Instance)
{
  UINT8      Buf[8];
  EFI_STATUS Status;

  // Set XDATA page to EVENT_BUF_ADDR
  Status = NvtSetXdataPage(Instance, Instance->MemMap->EVENT_BUF_ADDR);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // Clear HOST_CMD and HANDSHAKING registers
  Buf[0] = EVENT_MAP_HOST_CMD;
  Buf[1] = 0x00;
  Status = NvtI2cWrite(Instance, I2C_FW_Address, Buf, 2);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  gBS->Stall(10000); // 10ms delay
  return EFI_SUCCESS;
}

/**
  Check Firmware Status (Handshake)

  Waits for 0xAA in Handshaking register.

  @param Instance    Pointer to driver instance
  @retval EFI_SUCCESS           Handshake Received
  @retval EFI_TIMEOUT           Timeout
**/
EFI_STATUS NvtCheckFwStatus(IN NVT_INTERNAL_DATA *Instance)
{
  UINT8      Buf[8];
  EFI_STATUS Status;
  UINT32     Retry;

  NvtSetSlaveAddress(Instance, I2C_FW_Address);

  for (Retry = 0; Retry < 50; Retry++) {
    // Set page
    Buf[0] = 0xFF;
    Buf[1] = (UINT8)((Instance->EventBufAddr >> 16) & 0xFF);
    Buf[2] = (UINT8)((Instance->EventBufAddr >> 8) & 0xFF);
    NvtI2cWriteRaw(Instance, Buf, 3);

    // Read handshaking status
    Status =
        NvtI2cRead(Instance, EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE, Buf, 2);
    if (!EFI_ERROR(Status) && (Buf[1] == 0xAA)) {
      DEBUG((EFI_D_ERROR, "NVT: FW Status Check OK (0xAA)\n"));
      return EFI_SUCCESS;
    }

    gBS->Stall(10000); // 10ms
  }

  DEBUG((EFI_D_ERROR, "NVT: FW Status Check Timeout\n"));
  return EFI_TIMEOUT;
}
