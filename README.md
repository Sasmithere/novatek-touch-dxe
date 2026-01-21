

# novatek-touch-dxe
novatek touch dxe implementation using software i2c

now touch here is not perfect but works for basic touch (no left,right,up,down swipe work for now)
currently works for redmi note 7 pro (violet), all the default values are from violet , so check yours properly

# NovatekTouchDxe

UEFI Driver for Novatek NT36xxx Series Touchscreen Controllers (specifically hardened for NT36672A). This driver provides `EFI_ABSOLUTE_POINTER_PROTOCOL` support, enabling touch input in UEFI environments (Shell, Menus, Windows Boot Manager, etc.).

## Features

-   **Wide Compatibility**: Supports Original and Aftermarket (Clone) ICs.
-   **Robust Initialization**: Implements "Soft-Fail" logic to tolerate broken firmware states common in replacement screens.
-   **Ghost Touch Filtering**: Intelligent Band-Pass filter rejects high-pressure/narrow-width noise while passing valid touches.
-   **Software I2C (Bit-Banging)**: Bypasses hardware QUP limitations/instability by using direct GPIO manipulation for I2C communication.
-   **Dynamic Resolution**: Automatically detects firmware resolution or falls back to safe defaults (PCD defined) if firmware is corrupt.

## Integration Guide

### 1. Add to Platform DSC
Include the driver in your platform's `.dsc` file (e.g., `violet.dsc`) under `[Components.common]`:

```ini
[Components.common]
  Silicon/Qualcomm/QcomPkg/Drivers/NovatekTouchDxe/NovatekNvtTsDevice.inf
  Silicon/Qualcomm/QcomPkg/Drivers/NovatekTouchDxe/NovatekNvtTs.inf
```
The above needs to be in the order as stated , 1st NovatekNvtTsDevice , 2nd NovatekNvtTs

### 2. Add to Platform FDF
Include the driver binary in your platform's `.fdf` file (e.g., `violet.fdf`) inside the main Firmware Volume (usually `[FV.FvMain]`):

```ini
  INF Silicon/Qualcomm/QcomPkg/Drivers/NovatekTouchDxe/NovatekNvtTsDevice.inf
  INF Silicon/Qualcomm/QcomPkg/Drivers/NovatekTouchDxe/NovatekNvtTs.inf
```

### 3. Configure PCDs
Define the necessary PCDs in your platform's `.dsc` file to match your hardware configuration.

#### Critical Configuration
| PCD Name | Type | Description | Values (Example) |
| :--- | :--- | :--- | :--- |
| `PcdNvtTouchCtlrI2cDevice` | UINT32 | I2C Bus Instance (QUP ID) | `1` | #this implementation is for software i2c so i dont think this is useful here
| `PcdNvtTouchCtlrAddress` | UINT16 | I2C Slave Address (7-bit) | `0x62` |
| `PcdNvtTouchCtlrResetPin` | UINT32 | GPIO Pin for RESET | `88` |
| `PcdNvtTouchCtlrIntPin` | UINT32 | GPIO Pin for INTERRUPT | `89` |

#### Software I2C (Bit-Banging) Pins
Since this driver uses Software I2C, you **MUST** configure the SDA and SCL GPIOs:
| PCD Name | Type | Description | Values (Example) |
| :--- | :--- | :--- | :--- |
| `PcdNvtTouchGpioSda` | UINT32 | GPIO Pin for I2C SDA | `4` |
| `PcdNvtTouchGpioScl` | UINT32 | GPIO Pin for I2C SCL | `5` |

#### Resolution & Orientation
| PCD Name | Type | Description | Values (Example) |
| :--- | :--- | :--- | :--- |
| `PcdNvtTouchMaxX` | UINT32 | Screen Width | `1080` | 
| `PcdNvtTouchMaxY` | UINT32 | Screen Height | `2340` |
| `PcdNvtTouchInvertedX` | BOOLEAN | Invert X-Axis | `FALSE` |
| `PcdNvtTouchInvertedY` | BOOLEAN | Invert Y-Axis | `FALSE` |

### Example `.dsc` Configuration
```ini
[PcdsFixedAtBuild.common]
  # IO
  gNovatekTouchTokenSpaceGuid.PcdNvtTouchCtlrResetPin|88
  gNovatekTouchTokenSpaceGuid.PcdNvtTouchCtlrIntPin|89
  
  # Bit-Banging I2C Pins
  gNovatekTouchTokenSpaceGuid.PcdNvtTouchGpioSda|4
  gNovatekTouchTokenSpaceGuid.PcdNvtTouchGpioScl|5
  
  # Resolution
  gNovatekTouchTokenSpaceGuid.PcdNvtTouchMaxX|1080
  gNovatekTouchTokenSpaceGuid.PcdNvtTouchMaxY|2340
```

## How It Works

1.  **Entry Point**: `NvtInitialize` registers the Driver Binding Protocol.
2.  **Start**: `NvtAbsolutePointerDriverBindingStart` is called by the UEFI Core.
    *   It initializes GPIOs (Reset/Int/SDA/SCL).
    *   Performs a Hardware Reset ensuring the Reset Pin is held low for >10ms.
    *   Probes the I2C bus using Bit-Banging.
    *   Reads Firmware Info (Resolution, ID).
    *   Installs `EFI_ABSOLUTE_POINTER_PROTOCOL`.
3.  **Operation**: The driver uses a Polling Timer (approx. 1ms interval) to read touch data.
    *   It reads registers `0x00`-`0x3F` via I2C.
    *   Applies Ghost Touch Filtering.
    *   Updates the Absolute Pointer state.

## Troubleshooting

### Touch not working / "Device Error"
-   **Check I2C Pins**: Ensure `PcdNvtTouchGpioSda` and `PcdNvtTouchGpioScl` are correct for your device.**####very important###** 
-   **Check Power**: Ensure VDD/VCC lines are powered (usually by PMIC drivers loaded earlier).# in violets case, its hardware coded so it was not required here in this case
-   **Check Logs**: Enable `EFI_D_INFO` and `EFI_D_ERROR` in `Pkg` level `DebugLib` configuration to see detailed logs from `NovatekNvtTs`.

### Ghost Touches
-   The driver has built-in filtering. If ghost touches persist, check your grounding or try adjusting the Band-Pass filter width thresholds in `NovatekNvtTs.c` (`NvtGetTouchData`).

### Inverted Axis
-   If touches are mirrored, toggle `PcdNvtTouchInvertedX` or `PcdNvtTouchInvertedY` in your `.dsc`.

the 2nd most important part the "AI" missed here is the clocks required and how they are needed to be tuned properly, check your kernel online if available, or probe your phone using
adb , read the comments in the driver properly...

there have been issues where touch is not working , trust me the code is working , the thing i found when testing is its the timer, for now the working theory is the timing is set differently
on different devices, and since the time in the current dxe favours my phone so touch is working properly , you need to find yours by testing, more specifically, its "mI2cDelayUs", so dxe is perfect its
just a timing issue....

toodles
