// Blink the blue user LED on the CANBed M4.
#include "board.hpp"

using namespace std::chrono_literals;

int
main()
{
	Board::initialize();

	while (true)
	{
		Board::Led::toggle();
		modm::delay(250ms);
	}
	return 0;
}
