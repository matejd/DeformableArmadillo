#include "Timer.hpp"

#ifndef EMSCRIPTEN // WebGL 1.0 does not support syncs and queries!
#include <GL/glew.h>
#include <GL/glfw.h>

GpuTimer::GpuTimer()
{
}

void GpuTimer::start()
{
    QueryPair qp;
    if (availablePairs.empty()) {
        glGenQueries(1, &qp.startQuery);
        glGenQueries(1, &qp.endQuery);
        std::cout << "generated new queries" << std::endl;
    }
    else {
        qp = availablePairs.front();
        availablePairs.pop(); // Reuse old OpenGL ids, no need for additional glGenQueries.
    }

    inflightPairs.push(qp);
    ASSERT(inflightPairs.size() < 10);
    // Create a fence and insert it into the command stream. This fence will be signaled
    // when the GPU has completed all previously issued commands. glWaitSync will defer
    // subsequently issued commands (without blocking CPU) until the fence signals.
    // This prevents overlapped execution of OpenGL calls and should make timing measurements
    // more precise. For more, see e.g. OpenGL Insights.
    GLsync sync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    ASSERT(sync != nullptr);
    glWaitSync(sync, 0, GL_TIMEOUT_IGNORED);
    glDeleteSync(sync); // Flag for deletion.
    glQueryCounter(qp.startQuery, GL_TIMESTAMP);
}

double GpuTimer::elapsed()
{
    glQueryCounter(inflightPairs.back().endQuery, GL_TIMESTAMP);

    // Query just issued is not going to be available just yet.
    // We'll return the oldest available result.
    // On startup, there won't be any available.
    if (inflightPairs.size() == 1) {
        std::cout << "Just one query pair!" << std::endl;
        return 0.;
    }

    // Check if a previously issued query is available. Since startQuery
    // is issued before endQuery, we check endQuery only.
    QueryPair qp = inflightPairs.front();
    GLuint queryAvailable = GL_FALSE;
    glGetQueryObjectuiv(qp.endQuery, GL_QUERY_RESULT_AVAILABLE, &queryAvailable);
    if (queryAvailable) {
        availablePairs.push(qp);
        inflightPairs.pop();

        glGetQueryObjectuiv(qp.startQuery, GL_QUERY_RESULT_AVAILABLE, &queryAvailable);
        ASSERT(queryAvailable == GL_TRUE);

        GLuint64 startTime, endTime;
        glGetQueryObjectui64v(qp.startQuery, GL_QUERY_RESULT, &startTime);
        glGetQueryObjectui64v(qp.endQuery,   GL_QUERY_RESULT, &endTime);
        double duration = double(endTime-startTime) * 0.001 * 0.001; // Milliseconds.
        ASSERT(duration > 0.);
        return duration;
    }

    std::cout << "returning 0" << std::endl;
    return 0.;
}
#endif // EMSCRIPTEN
