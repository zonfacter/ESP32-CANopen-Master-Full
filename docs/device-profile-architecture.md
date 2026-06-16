# Enhancement: Device Profiles for CANopen and RS485 Devices

## Goal

The project should be able to support many different field devices without wiring every device-specific behavior directly into the main sketch or UI callbacks.

Typical tasks are:

- scan a bus and find reachable participants
- identify a device
- show diagnostic information
- read or change Node ID / device address
- read or change baud rate
- run safe device-specific parameterization sequences

The first target is CANopen, but the design should leave room for RS485 devices later, for example Modbus RTU or vendor-specific ASCII protocols.

## Current Pain Point

The project already supports several device families, but the handling is still partly hard-coded:

- known device classification uses `KnownDeviceType`
- Node detail actions dispatch via conditionals
- device-specific baud or setup sequences can grow inside the main `.ino`
- UI actions are not yet driven by per-device capabilities

This works for a few devices, but becomes hard to maintain when more CANopen and RS485 devices are added.

## Proposed Direction

Introduce a small `DeviceProfile` / adapter layer.

A device profile describes:

- how the device is recognized
- which bus/protocol it uses
- which actions are supported
- how safety-sensitive actions must be executed
- which diagnostics should be shown

This should start small and table-driven. It does not need to become a large plugin framework.

## Core Data Model

```cpp
enum class BusType : uint8_t {
    CANopen,
    RS485_ModbusRTU,
    RS485_ASCII,
};

enum DeviceCapability : uint32_t {
    DEV_CAP_READ_IDENTITY = 1UL << 0,
    DEV_CAP_READ_DIAG     = 1UL << 1,
    DEV_CAP_SET_NODE_ID   = 1UL << 2,
    DEV_CAP_SET_BAUD      = 1UL << 3,
    DEV_CAP_RESET_COMM    = 1UL << 4,
    DEV_CAP_SAVE_PARAMS   = 1UL << 5,
};

struct DeviceIdentity {
    BusType bus;
    uint8_t address;
    uint32_t baud;

    // CANopen identity, if available.
    uint32_t vendorId;
    uint32_t productCode;
    uint32_t revision;
    uint32_t serial;

    // RS485/protocol-specific fields can be added later.
    uint32_t deviceCode;
};

struct DeviceProfile {
    const char* name;
    BusType bus;
    uint32_t capabilities;

    bool (*match)(const DeviceIdentity& id);
    void (*readDiagnostics)(DeviceContext& ctx);
    void (*setNodeId)(DeviceContext& ctx, uint8_t newId);
    void (*setBaud)(DeviceContext& ctx, uint32_t baud);
};
```

`DeviceContext` would wrap the active transport and UI/log helpers. For CANopen this can expose an `SdoClient`, NMT helper and current node ID. For RS485 it can expose a serial transport, address, baud and protocol settings.

## Discovery Result

Scanners should produce a common discovery result independent of bus type:

```cpp
struct DiscoveredDevice {
    BusType bus;
    uint8_t address;
    uint32_t baud;
    const DeviceProfile* profile;
    DeviceIdentity identity;
    char displayName[32];
};
```

The UI can then show one unified list:

- bus type
- address / Node ID
- baud rate
- recognized profile name
- basic status

## CANopen Scanner

Initial CANopen scanner behavior:

- iterate configured baud rates
- actively probe Node IDs, for example SDO read `0x1000:00`
- collect SDO responses
- read identity object `0x1018`
- classify with registered `DeviceProfile::match`
- optionally support LSS/Fastscan later

Useful generic CANopen diagnostics:

- `0x1000:00` device type
- `0x1001:00` error register
- `0x1018:01..04` identity
- `0x1003` predefined error field, if supported
- `0x603F` DS402 error code, if supported
- `0x6041` DS402 statusword, if supported
- last EMCY frame, if seen by sniffer

## Standard CANopen Profiles

The profile layer should not only support known vendor-specific devices. It should also provide useful fallback profiles for common CANopen standards, so that unknown devices are at least found, identified and shown with basic diagnostics.

### DS301 / CiA 301 Base Profile

Every CANopen device that responds to SDO should get a safe DS301 page.

Minimum discovery:

- SDO ping via `0x1000:00`
- identity via `0x1018:01..04`
- device type decode from `0x1000:00`
- error register via `0x1001:00`
- predefined error field `0x1003`, if present
- heartbeat / boot-up observation, if seen
- EMCY display, if seen by sniffer

Minimum actions:

- NMT Start
- NMT Pre-Operational
- NMT Stop
- Reset Node
- Reset Communication

No device-specific writes should be attempted for unknown DS301 devices.

### DS402 / CiA 402 Drives

If the device type or object dictionary suggests a drive profile, provide a DS402 diagnostic page even when the vendor is unknown.

Useful probes:

- `0x6040` controlword, read if supported
- `0x6041` statusword
- `0x603F` error code
- `0x6060` modes of operation
- `0x6061` modes of operation display
- `0x6064` position actual value, if supported

Safe initial behavior:

- read-only diagnostics by default
- optional fault reset / state-machine controls only behind an explicit DS402 action profile
- no motion-related writes for unknown drives

### CiA 401 I/O Modules

CiA 401 devices should be found and displayed even without a vendor-specific profile.

Useful probes:

- device type from `0x1000:00`
- identity from `0x1018`
- error register `0x1001`
- digital inputs, commonly around `0x6000`
- digital outputs, commonly around `0x6200`, read-only by default
- analog inputs/outputs, commonly around `0x6401` and related ranges, if present

Safe initial behavior:

- show available input values
- show output objects as read-only first
- require an explicit profile or confirmation before writing outputs

### CiA 404 Sensors And Controllers

CiA 404 devices should be detected as sensor/controller class devices and shown with generic diagnostics.

Useful probes:

- device type from `0x1000:00`
- identity from `0x1018`
- error register `0x1001`
- sensor values through profile-specific object ranges where present

Safe initial behavior:

- read-only value display
- no calibration or controller parameter writes without a vendor-specific profile

### CiA 406 Encoders

CiA 406 encoders should get a simple encoder detail page.

Useful probes:

- device type from `0x1000:00`
- identity from `0x1018`
- position value, commonly `0x6004`
- operating parameters, commonly `0x6000`
- resolution / measuring units, if present
- preset value object, if present, read-only by default

Safe initial behavior:

- show actual position
- show encoder status/diagnostics
- do not write preset or scaling objects unless a specific profile enables it

These standard profiles should be registered before the final "Unknown CANopen" fallback. Vendor-specific profiles can override them when identity matching is known.

## RS485 Scanner

RS485 support should be added as a separate scanner rather than mixing it into CANopen code.

Possible scan dimensions:

- baud rate
- parity
- stop bits
- address range
- protocol probe

For Modbus RTU:

- try common baud/parity combinations
- scan address range
- read a safe identification or status register, configurable per profile

For vendor-specific ASCII:

- profile-specific probe command
- configurable response timeout
- parser callback inside the profile

## UI Behavior

The node/device detail UI should be capability-driven:

- show "Read Info" if `DEV_CAP_READ_IDENTITY`
- show "Read Diagnostics" if `DEV_CAP_READ_DIAG`
- show "Set Node ID" if `DEV_CAP_SET_NODE_ID`
- show "Set Baud" if `DEV_CAP_SET_BAUD`
- show reset/save controls only if the profile supports them

Unknown devices should still get a safe standard page:

- CANopen: NMT controls, identity, error register, raw SDO read helper later
- RS485: protocol/address/baud info, safe probe result, raw diagnostics later

## Safety Rules

Profiles should explicitly encode safety requirements for write operations.

Examples:

- CANopen NMT Pre-Operational required
- Rexroth communication phase 2 required
- drive enable must be off
- parameter must be saved separately
- power-cycle required after write
- do not write if current encoding cannot be confirmed by readback

For safety-sensitive devices, the profile should implement a small state machine rather than a single direct write.

The Bosch Rexroth ECODRIVE baud-rate workflow is a good example:

- read current parameter first
- verify object and encoding
- verify safe parameterization phase
- run documented transition command if needed
- write only after guards are satisfied
- log detailed diagnostics if blocked

## Prior Art And Possible Reuse

Several open-source CANopen projects can inform this architecture. They should be treated differently depending on license and target fit.

### CANopenNode

Repository: <https://github.com/CANopenNode/CANopenNode>

CANopenNode is an Apache-2.0 CANopen protocol stack written in ANSI C. It includes many building blocks that are relevant to this project:

- DS301 / CiA 301 base stack
- NMT handling
- heartbeat / node guarding
- SDO server and SDO client
- EMCY
- PDO mapping concepts
- LSS master/slave and LSS fastscan
- CiA 309-3 gateway / ASCII command ideas
- non-volatile object dictionary storage concepts
- object dictionary structure and editor workflow

Potential use:

- reference implementation for protocol behavior and edge cases
- possible future replacement or optional backend for parts of the CANopen stack
- reference for LSS fastscan and SDO client robustness
- reference for object dictionary and EDS/XDD tooling

Integration note:

- Direct integration should be evaluated carefully because the current project already has a small custom CANopen master layer. A full stack may add size and complexity on ESP32.
- Short-term: use as reference and test oracle.
- Long-term: consider importing selected Apache-2.0-compatible components only if the local implementation becomes too costly to maintain.

### MHS TinyCanOpen

Repository: <https://github.com/MHS-Elektronik/TinyCanOpen>

TinyCanOpen is a virtual CANopen device based on CANopenNode with a GUI. It is Apache-2.0 licensed.

Potential use:

- host-side simulation device for scan / identify / SDO tests
- regression test companion for DS301 behavior
- inspiration for a small virtual device fixture
- helpful for testing without physical CANopen hardware attached

Integration note:

- This is not a firmware component for the ESP32 target.
- It is useful as external test tooling, especially if combined with SLCAN/Tiny-CAN compatible hardware or a PC-side CAN interface.

### MHS CanOpenMonitor

Repository: <https://github.com/MHS-Elektronik/CanOpenMonitor>

CanOpenMonitor is a CANopen logging and injection tool. It provides useful architectural ideas:

- plugin model for protocol decoding and actions
- NMT plugin
- SDO editor
- EDS-based object dictionary display
- DCF save/load concepts
- EEPROM plugin for `0x1010` / `0x1011`
- human-readable SDO/PDO decoding
- warning that live bus interaction can be dangerous

Potential use:

- UI and workflow inspiration
- EDS/DCF concept reference
- useful external PC tool during development and debugging

License note:

- CanOpenMonitor is GPL-3.0. Do not copy code into this firmware project unless the project intentionally accepts GPL obligations.
- Safe use: study concepts, use as an external tool, or document compatible workflows.

## Suggested File Layout

```text
SIKO_AP10_Test/
  src/
    bus/
      bus_types.h
      bus_scanner.h
      canopen_scanner.h
      rs485_scanner.h
    devices/
      device_profile.h
      device_registry.h
      profiles/
        siko_ap04_profile.h
        siko_ap10_profile.h
        dunker_75ci_profile.h
        rexroth_ecodrive_profile.h
    ui/
      device_detail_ui.h
```

The first implementation can be smaller than this. The important part is the dependency direction:

- scanners discover devices
- registry classifies devices
- UI asks the profile what actions are available
- profile implements device-specific sequences

## Migration Plan

### Step 1: Add Profile Registry

- add `DeviceProfile`
- add `DeviceIdentity`
- add `DeviceRegistry`
- keep `KnownDeviceType` for compatibility at first
- register current known devices in one table

Acceptance:

- current identify flow still works
- UI still shows SIKO / Dunker / Rexroth names
- no behavior change yet

### Step 2: Move Classification Into Profiles

- replace hard-coded `classifyByIdentity` conditionals with profile match callbacks
- keep a fallback "Unknown CANopen" profile
- add standard fallback profiles for DS301, DS402, CiA 401, CiA 404 and CiA 406 based on `0x1000:00` device type and safe object probes

Acceptance:

- Identify All classifies via registry
- unknown devices still open the standard page
- common standard devices are at least recognized as drive / I/O / sensor / encoder class when possible

### Step 3: Capability-Driven Node Detail Actions

- Node detail UI reads capabilities from the active profile
- disable or hide unsupported actions
- route "Set Baud", "Set Node ID", "Read Diagnostics" through profile callbacks

Acceptance:

- SIKO, Dunker and Rexroth existing actions still work
- unsupported devices no longer show misleading controls

### Step 4: Move Device-Specific State Machines Out Of The Main Sketch

- move Rexroth ECODRIVE baud state machine into a profile/action class
- move Dunker-specific behavior into a profile
- keep the main sketch responsible only for dispatch and UI refresh

Acceptance:

- main `.ino` becomes smaller
- adding a new CANopen device does not require editing unrelated UI logic

### Step 5: Add RS485 Transport And Scanner

- add RS485 serial transport abstraction
- add first RS485 scan mode
- add one simple RS485 device profile

Acceptance:

- CANopen scan remains unchanged
- RS485 scan results appear in the same discovered device list

## Open Questions

- Should unsupported profile actions be hidden or shown disabled with a reason?
- Should scan results be stored in RAM only or exportable as a report?
- Which RS485 protocol should be first: Modbus RTU or vendor-specific ASCII?
- Should EDS/DCF import be attempted on-device, or only via generated profile tables?
- How much raw expert access should the UI expose for unknown devices?

## Definition of Done

- New CANopen devices can be added by writing one profile and registering it.
- Standard CANopen devices using DS301, DS402, CiA 401, CiA 404 or CiA 406 are found by scan and get safe read-only diagnostics where possible.
- Device detail UI is driven by capabilities, not hard-coded device switches.
- Existing SIKO, Dunker and Rexroth behavior remains functional.
- Unknown CANopen devices still get a safe standard page.
- Architecture leaves a clear path for RS485 scanning and parameterization.
