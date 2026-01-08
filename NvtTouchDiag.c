#include <Library/DebugLib.h>
#include <Library/ShellCEntryLib.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

// ==================== PROTOCOL DEFINITION (Duplicated) ====================
typedef struct {
  UINT32 DroppedEvents;
  UINT32 MaxPollDuration;
  UINT32 I2cRetries;
  UINT32 EventBufAddr;
  UINT8  CurrentMode;
  INT16  LastPacketId;
  UINT16 FirmwareVersion;
  UINT16 ResolutionX;
  UINT16 ResolutionY;
  UINT64 LastPollTime;
} NOVATEK_TOUCH_DIAGNOSTICS;

typedef struct _NOVATEK_TOUCH_CONFIG_PROTOCOL NOVATEK_TOUCH_CONFIG_PROTOCOL;

typedef EFI_STATUS(EFIAPI *NVT_GET_DIAGNOSTICS)(
    IN NOVATEK_TOUCH_CONFIG_PROTOCOL *This,
    OUT NOVATEK_TOUCH_DIAGNOSTICS    *Diagnostics);

typedef EFI_STATUS(EFIAPI *NVT_SET_I2C_DELAY)(
    IN NOVATEK_TOUCH_CONFIG_PROTOCOL *This, IN UINT32 DelayUs);

typedef EFI_STATUS(EFIAPI *NVT_RESET_CONTROLLER)(
    IN NOVATEK_TOUCH_CONFIG_PROTOCOL *This);

struct _NOVATEK_TOUCH_CONFIG_PROTOCOL {
  NVT_GET_DIAGNOSTICS  GetDiagnostics;
  NVT_SET_I2C_DELAY    SetI2cDelay;
  NVT_RESET_CONTROLLER ResetController;
};

extern EFI_GUID gNovatekTouchConfigProtocolGuid;

// =================================================================

INTN EFIAPI ShellAppMain(IN UINTN Argc, IN CHAR16 **Argv)
{
  EFI_STATUS                     Status;
  NOVATEK_TOUCH_CONFIG_PROTOCOL *NvtConfig;
  NOVATEK_TOUCH_DIAGNOSTICS      Diag;

  Print(L"Novatek Touch Diagnostic Tool\n");

  Status = gBS->LocateProtocol(
      &gNovatekTouchConfigProtocolGuid, NULL, (VOID **)&NvtConfig);

  if (EFI_ERROR(Status)) {
    Print(
        L"Error: Novatek Touch Driver NOT found or Protocol not installed.\n");
    return Status;
  }

  if (Argc > 1) {
    if (StrCmp(Argv[1], L"reset") == 0) {
      Print(L"Resetting Controller...\n");
      Status = NvtConfig->ResetController(NvtConfig);
      Print(L"Reset Status: %r\n", Status);
      return Status;
    }

    if (StrCmp(Argv[1], L"delay") == 0) {
      if (Argc < 3) {
        Print(L"Usage: NvtTouchDiag delay <us> (e.g. 5)\n");
        return EFI_INVALID_PARAMETER;
      }
      UINTN Delay = StrDecimalToUintn(Argv[2]);
      Print(L"Setting I2C Delay to %d us...\n", Delay);
      Status = NvtConfig->SetI2cDelay(NvtConfig, (UINT32)Delay);
      Print(L"Status: %r\n", Status);
      return Status;
    }
  }

  // Default: Dump Diagnostics
  Print(L"--- Driver Diagnostics ---\n");
  Status = NvtConfig->GetDiagnostics(NvtConfig, &Diag);
  if (EFI_ERROR(Status)) {
    Print(L"Failed to get diagnostics: %r\n", Status);
    return Status;
  }

  Print(L"FW Ver: 0x%02x\n", Diag.FirmwareVersion);
  Print(L"Res:    %dx%d\n", Diag.ResolutionX, Diag.ResolutionY);
  Print(L"Mode:   0x%02x\n", Diag.CurrentMode);
  Print(L"EventBuf: 0x%X\n", Diag.EventBufAddr);
  Print(L"Dropped Events: %d\n", Diag.DroppedEvents);
  Print(L"I2C Retries:    %d\n", Diag.I2cRetries);
  Print(L"Max Poll Time:  %d us\n", Diag.MaxPollDuration);
  Print(L"Last Packet ID: %d\n", Diag.LastPacketId);

  return EFI_SUCCESS;
}
