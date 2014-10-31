#ifndef Timer_Hpp
#define Timer_Hpp

#include "Platform.hpp"
#include <chrono>

class CpuTimer
{
private:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;
    TimePoint beginning;

public:
    void start()
    {
        beginning = std::chrono::high_resolution_clock::now();
    }

    /// Returns the amount of milliseconds (nanosecond precision) elapsed since last start.
    double elapsed()
    {
        const TimePoint now = std::chrono::high_resolution_clock::now();
        const double duration = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now-beginning).count()) * 0.001 * 0.001;
        return duration;
    }

    /// Resets the timer and returns the amount of milliseconds (nanosecond precision) elapsed since last start.
    double restart()
    {
        const TimePoint now = std::chrono::high_resolution_clock::now();
        const double duration = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now-beginning).count()) * 0.001 * 0.001;
        beginning = now;
        return duration;
    }
};

#ifndef EMSCRIPTEN
/// GPU timing. Has the same interface as the CpuTimer, but behaves a bit differently due to the asynchronous nature
/// of the GPUs. GpuTimer must *not* be used as a temporary variable. Create it once on startup and
/// then call start() and elapsed() every frame around OpenGL calls you want to time. Use
/// several GpuTimers if you want to time different sequence of OpenGL calls. This is because
/// of the latency involved: elapsed() returns the oldest available measurement, which might
/// be several frames behind.
class GpuTimer
{
public:
    GpuTimer();

    /// The next OpenGL call will be the first one timed.
    void start();

    /// Returns *oldest available* measurement. The duration returned might be several frames behind.
    double elapsed();

private:
    struct QueryPair
    {
        u32 startQuery, endQuery;
    };

    Queue<QueryPair> inflightPairs; // Pairs of queries we're waiting on.
    Queue<QueryPair> availablePairs; // Freed pairs.
};
#endif // EMSCRIPTEN

/// The purpose of SampleStats is to simplify common operations on profiling data (e.g. code segment execution times).
template <typename T, int SIZE=50>
class SampleStats
{
public:
    T average, min, max;

    /// Allocates sample of size SIZE. Stats (average, min, max) are invalid until sample has been fully filled up (after
    /// that adding new data will remove the oldest from the sample).
    SampleStats(): average(T(0)), min(T(0)), max(T(0)), data(SIZE, T(0)), currentIndex(0) { ASSERT(data.size() == SIZE); }

    /// Adds value to the sample data, removing the oldest value in the data if necessary (when sample size is over SIZE).
    /// Also updates public statistics (average, min, max).
    void add(const T& value)
    {
        if (currentIndex == SIZE)
            currentIndex = 0;
        data[currentIndex] = value;
        currentIndex++;

        T sum = T(0);
        min = data[0];
        max = data[0];
        for (int i = 0; i < SIZE; ++i) {
            const T& v = data[i];
            sum += v;
            if (v < min) min = v;
            if (max < v) max = v;
        }
        average = sum / SIZE;
    }

private:
    Vector<T> data;
    int currentIndex;
};

#endif // Timer_Hpp
