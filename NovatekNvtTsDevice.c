#include <Uefi.h>

#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/AbsolutePointer.h>
#include <Protocol/EFIClock.h>
#include <Protocol/EFII2C.h>
#include <Protocol/EFITlmm.h>

#include "NovatekNvtTs.h"
#include <Device/TouchDevicePath.h>

NOVATEK_I2C_DEVICE mNvtTemplate = {
    .Signature              = NVT_DEV_INSTANCE_SIGNATURE,
    .XMax                   = 0,
    .YMax                   = 0,
    .XMin                   = 0,
    .YMin                   = 0,
    .XInverted              = FALSE,
    .YInverted              = FALSE,
    .ControllerResetPin     = 0,
    .ControllerInterruptPin = 0,
    .ControllerI2cDevice    = 0,
    .GpioTlmmProtocol       = NULL,
    .I2cQupProtocol         = NULL,
    .SlaveCfg =
        {
            .BusFrequency = I2C_FAST_MODE_FREQ_KHZ,
            .SlaveAddress = I2C_HW_Address, // Default to HW Address (0x62)
            .Mode         = I2C,
            .SlaveMaxClockStretch = 500,
            .CoreConfiguration1   = 0,
            .CoreConfiguration2   = 0,
        },
};

EFI_STATUS EFIAPI
NvtDeviceInitialize(IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable)
{
  NOVATEK_I2C_DEVICE *Instance;
  EFI_STATUS          Status;

  // Device instance
  Instance = AllocateCopyPool(sizeof(NOVATEK_I2C_DEVICE), &mNvtTemplate);
  if (Instance == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto exit;
  }

  // Config
  // Hardcoded from sm6150-novatek-i2c_f7b.dtsi
  // Config from PCDs (NovatekTouch.dec)
  Instance->SlaveCfg.SlaveAddress  = PcdGet16(PcdNvtTouchCtlrAddress);
  Instance->ControllerResetPin     = PcdGet32(PcdNvtTouchCtlrResetPin);
  Instance->ControllerInterruptPin = PcdGet32(PcdNvtTouchCtlrIntPin);
  Instance->ControllerI2cDevice    = PcdGet32(PcdNvtTouchCtlrI2cDevice);
  Instance->ControllerVddPin       = PcdGet32(PcdNvtTouchCtlrVddPin);
  Instance->ControllerVddIoPin     = PcdGet32(PcdNvtTouchCtlrVddIoPin);
  Instance->ControllerVddCtlActiveLow =
      PcdGetBool(PcdNvtTouchCtlrVddPinActiveLow);

  Instance->XMax = 1080; // Hard fallback
  Instance->YMax = 2340;

  // Allow PCD override for resolution if set
  if (PcdGet32(PcdNvtTouchMaxX) != 0)
    Instance->XMax = PcdGet32(PcdNvtTouchMaxX);
  if (PcdGet32(PcdNvtTouchMaxY) != 0)
    Instance->YMax = PcdGet32(PcdNvtTouchMaxY);

  Instance->XMin      = PcdGet32(PcdNvtTouchMinX);
  Instance->XInverted = PcdGetBool(PcdNvtTouchInvertedX);
  Instance->YMin      = PcdGet32(PcdNvtTouchMinY);
  Instance->YInverted = PcdGetBool(PcdNvtTouchInvertedY);

  DEBUG(
      (EFI_D_INFO,
       "NovatekNvtTsDevice: "
       "Address: 0x%X Device: %d "
       "ResetPin: %d IntPin: %d "
       "X: %d - %d (Inverted: %d) "
       "Y: %d - %d (Inverted: %d)\n",
       Instance->SlaveCfg.SlaveAddress, Instance->ControllerI2cDevice,
       Instance->ControllerResetPin, Instance->ControllerInterruptPin,
       Instance->XMin, Instance->XMax, Instance->XInverted, Instance->YMax,
       Instance->YMax, Instance->YInverted));

  // I2C Protocol
  Status = gBS->LocateProtocol(
      &gQcomI2cProtocolGuid, NULL, (VOID *)&Instance->I2cQupProtocol);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR, "NovatekNvtTsDevice: Unable to locate I2C protocol: %r\n",
         Status));
    goto exit;
  }

  // GPIO Processing
  Status = gBS->LocateProtocol(
      &gQcomTlmmProtocolGuid, NULL, (VOID *)&Instance->GpioTlmmProtocol);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR,
         "NovatekNvtTsDevice: Unable to locate GPIO protocol: %r\n", Status));
    goto exit;
  }

  // Looks good and publish the protocol
  Status = gBS->InstallMultipleProtocolInterfaces(
      &ImageHandle, &gNovatekTouchDeviceProtocolGuid, Instance,
      &gEfiDevicePathProtocolGuid, &TouchDxeDevicePath, NULL);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (EFI_D_ERROR, "NovatekNvtTsDevice: Unable to install protocol: %r\n",
         Status));
  }

exit:
  return Status;
}
