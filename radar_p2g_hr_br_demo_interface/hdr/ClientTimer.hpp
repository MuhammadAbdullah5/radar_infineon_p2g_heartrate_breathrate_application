#include <cstdint>
#include <chrono>

namespace ClientTimerModule {

	class Timer {

		clock_t start;

	public:
		Timer();
		~Timer();

		void StartTimer();
		void EndTimer();

	};
}