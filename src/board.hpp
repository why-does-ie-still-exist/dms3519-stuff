// Board support for the Longan Labs CANBed M4 (ATSAME51G19A, QFN48).
//
// Pin assignments come from the schematic (Longan-Labs/Hardware_CANBed_Series)
// and the Arduino variant (LONGAN-SAME-TOOLS variants/CANBED_M4).
#pragma once

#include <modm/platform.hpp>

namespace Board
{
using namespace modm::literals;
using namespace modm::platform;

/// The board has no crystal.  After reset the SAME51 runs from the DFLL48M in
/// open-loop mode, which is what we keep: 48 MHz core clock, no PLL.
struct SystemClock
{
	static constexpr uint32_t Frequency = GenericClockController::BootFrequency;  // 48 MHz
	static constexpr uint32_t Usb = Frequency;

	static bool inline
	enable()
	{
		GenericClockController::setFlashLatency<Frequency>();
		GenericClockController::updateCoreFrequency<Frequency>();
		return true;
	}
};

// User LED D5 (blue), active high.  Arduino D13 / LED_BUILTIN.
using Led = GpioA07;

// CAN0 on PA22 (TX) / PA23 (RX), peripheral function I.  MCP2542FD transceiver
// with STBY tied to ground; the red TX/RX LEDs sit on the same lines.
using CanTx = GpioA22;
using CanRx = GpioA23;

// Serial1: SERCOM3, PA16 RX / PA17 TX (Grove connector J11 and header J12).
using UartTx = GpioA17;
using UartRx = GpioA16;

// USB: PA24 D-, PA25 D+.
using UsbDm = GpioA24;
using UsbDp = GpioA25;

inline void
initialize()
{
	SystemClock::enable();
	SysTickTimer::initialize<SystemClock>();

	Led::setOutput(modm::Gpio::Low);
}

}  // namespace Board
