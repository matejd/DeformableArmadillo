#include "App.hpp"
#include "Renderer.hpp"

#include <GL/glew.h>
#include <GL/glfw.h> // Using version 2.7.

#ifdef EMSCRIPTEN
#include <emscripten/emscripten.h>
#endif

namespace {
    App* gApp = nullptr;
    Vector<EventListener*> gEventListeners;
}

void glfwOnMousePos(int x, int y)
{
    MouseEvent event;
    event.button = MouseButton::Unknown;
    event.action = MouseAction::Move;
    event.mouseX = x;
    event.mouseY = y;

    for (EventListener* listener: gEventListeners) {
        if (listener->onEvent(event))
            break;
    }
}

void glfwOnMouseButton(int button, int action)
{
    MouseEvent event;
    switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:   event.button = MouseButton::Left;   break;
        case GLFW_MOUSE_BUTTON_RIGHT:  event.button = MouseButton::Right;  break;
        case GLFW_MOUSE_BUTTON_MIDDLE: event.button = MouseButton::Middle; break;
        default:
            ASSERT(false); return;
    }

    switch (action) {
        case GLFW_PRESS:   event.action = MouseAction::Press;   break;
        case GLFW_RELEASE: event.action = MouseAction::Release; break;
        default:
            ASSERT(false); return;
    };

    glfwGetMousePos(&event.mouseX, &event.mouseY);

    for (EventListener* listener: gEventListeners) {
        if (listener->onEvent(event))
            break;
    }
}

void glfwOnMouseWheel(int pos)
{
    static int previousWheelPos = 0;
    MouseEvent event;
    event.button = MouseButton::Unknown;
    event.action = MouseAction::Scroll;
    // mouseX and mouseY are repurposed for
    // two-dimensional scroll offsets.
    // Horizontal offset currently unsupported.
    event.mouseX = 0;
    event.mouseY = pos-previousWheelPos;
    previousWheelPos = pos;

    for (EventListener* listener: gEventListeners) {
        if (listener->onEvent(event))
            break;
    }
}

void glfwOnKey(int key, int action)
{
    KeyboardEvent event;
    switch (key) {
        case 'S':            event.key = Key::S;      break;
        case GLFW_KEY_SPACE: event.key = Key::Space;  break;
        case GLFW_KEY_ESC:   event.key = Key::Escape; break;
        case GLFW_KEY_ENTER: event.key = Key::Enter;  break;
        default:
            ASSERT(false); return;
    };

    switch (action) {
        case GLFW_PRESS:   event.action = KeyAction::Press;   break;
        case GLFW_RELEASE: event.action = KeyAction::Release; break;
        default:
            ASSERT(false); return;
    };

    for (EventListener* listener: gEventListeners) {
        if (listener->onEvent(event))
            break;
    }
}

void glfwOnChar(int key, int action)
{
}

void emscDrawFrame()
{
    ASSERT(gApp != nullptr);
    gApp->drawFrame();
}

App::App(int argc, char** argv)
{
}

App::~App()
{
}

void App::addEventListener(EventListener* listener)
{
    gEventListeners.push_back(listener);
}

int App::exec()
{
    if (glfwInit() != GL_TRUE) {
        std::cout << "Failed to init glfw!" << std::endl;
        return 1;
    }

    glfwOpenWindowHint(GLFW_WINDOW_NO_RESIZE, GL_TRUE);
    if (glfwOpenWindow(1280,720, 8,8,8,0, 24,0,GLFW_WINDOW) != GL_TRUE) {
        std::cout << "Failed to open a window!" << std::endl;
        glfwTerminate();
        return 2;
    }

    gApp = this;

    glfwGetWindowSize(&canvasWidth, &canvasHeight);

    glewInit();
    glfwSetWindowTitle("Deformable Armadillo");
    glfwSetKeyCallback(glfwOnKey);
    glfwSetCharCallback(glfwOnChar);
    glfwSetMousePosCallback(glfwOnMousePos);
    glfwSetMouseButtonCallback(glfwOnMouseButton);
    glfwSetMouseWheelCallback(glfwOnMouseWheel);

    glEnable(GL_DEPTH_TEST);

    if (!setup()) {
        glfwTerminate();
        return 3;
    }

#ifdef EMSCRIPTEN
    emscripten_set_main_loop(emscDrawFrame, 0, 1);
#else
    while (true) {
        drawFrame();
        glfwSwapBuffers();

        if (glfwGetKey(GLFW_KEY_ESC) || !glfwGetWindowParam(GLFW_OPENED))
            break;
    }
#endif

    std::cout << "Terminating..." << std::endl;
    glfwTerminate();
    return 0;
}
