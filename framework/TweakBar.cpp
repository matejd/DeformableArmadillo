#include "TweakBar.hpp"

#ifdef EMSCRIPTEN
// Not yet implemented. Maybe AntTweakBar works?
TweakBar::TweakBar() {}
TweakBar::~TweakBar() {}
void TweakBar::checkInit() {}
void TweakBar::addFloat(const String& name, float* ptrToVar) {}
void TweakBar::addFloat(const String& name, float* ptrToVar, const float min, const float max, const float step) {}
void TweakBar::addInt(const String& name, int* ptrToVar) {}
void TweakBar::addInt(const String& name, int* ptrToVar, const int min, const int max, const int step) {}
void TweakBar::addBool(const String& name, bool* ptrToVar) {}
void TweakBar::draw() {}
bool TweakBar::onEvent(const KeyboardEvent) { return false; }
bool TweakBar::onEvent(const MouseEvent event) { return false; }
#else

#include <AntTweakBar.h>
#include <sstream>

TweakBar::TweakBar()
{
}

TweakBar::~TweakBar()
{
    if (twBar != nullptr)
        TwTerminate();
}

void TweakBar::checkInit()
{
    if (twBar == nullptr) {
        TwInit(TW_OPENGL, nullptr);
        TwWindowSize(canvasWidth, canvasHeight);
        twBar = TwNewBar("TweakBar");
    }
}

void TweakBar::addFloat(const String& name, float* ptrToVar)
{
    checkInit();
    TwAddVarRO(twBar, name.c_str(), TW_TYPE_FLOAT, ptrToVar, "");
}

void TweakBar::addFloat(const String& name, float* ptrToVar, const float min, const float max, const float step)
{
    checkInit();

    std::ostringstream ss;
    ss << " min=" << min
       << " max=" << max
       << " step=" << step
       << " ";
    TwAddVarRW(twBar, name.c_str(), TW_TYPE_FLOAT, ptrToVar, ss.str().c_str());
}

void TweakBar::addInt(const String& name, int* ptrToVar)
{
    checkInit();
    TwAddVarRO(twBar, name.c_str(), TW_TYPE_INT32, ptrToVar, "");
}

void TweakBar::addInt(const String& name, int* ptrToVar, const int min, const int max, const int step)
{
    checkInit();

    std::ostringstream ss;
    ss << " min=" << min
       << " max=" << max
       << " step=" << step
       << " ";
    TwAddVarRW(twBar, name.c_str(), TW_TYPE_INT32, ptrToVar, ss.str().c_str());
}

void TweakBar::addBool(const String& name, bool* ptrToVar)
{
    checkInit();
    TwAddVarRW(twBar, name.c_str(), TW_TYPE_BOOLCPP, ptrToVar, "");
}

void TweakBar::draw()
{
    if (twBar != nullptr)
        TwDraw();
}

bool TweakBar::onEvent(const KeyboardEvent)
{
    return false;
}

bool TweakBar::onEvent(const MouseEvent event)
{
    checkInit();

    if (event.action == MouseAction::Move) {
        return TwMouseMotion(event.mouseX, event.mouseY);
    }

    if (event.action == MouseAction::Scroll)
        return false;

    TwMouseAction twAction;
    if (event.action == MouseAction::Press)
        twAction = TW_MOUSE_PRESSED;
    else if (event.action == MouseAction::Release)
        twAction = TW_MOUSE_RELEASED;
    else
        assert(false);

    TwMouseButtonID twButton;
    if (event.button == MouseButton::Left)
        twButton = TW_MOUSE_LEFT;
    else if (event.button == MouseButton::Middle)
        twButton = TW_MOUSE_MIDDLE;
    else if (event.button == MouseButton::Right)
        twButton = TW_MOUSE_RIGHT;
    else
        assert(false);

    return TwMouseButton(twAction, twButton);
}

#endif // EMSCRIPTEN
