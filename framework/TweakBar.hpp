#ifndef TweakBar_Hpp
#define TweakBar_Hpp

#include "Events.hpp"
#include "Platform.hpp"

struct CTwBar;
typedef struct CTwBar TwBar;

class TweakBar: public EventListener
{
public:
    int canvasWidth = 1024;
    int canvasHeight = 720;

    TweakBar();
    ~TweakBar();

    void addFloat(const String& name, float* ptrToVar); // Read-only.
    void addFloat(const String& name, float* ptrToVar, const float min, const float max, const float step);
    void addInt(const String& name, int* ptrToVar);
    void addInt(const String& name, int* ptrToVar, const int min, const int max, const int step);

    void addBool(const String& name, bool* ptrToVar); // RW.

    void draw();

    virtual bool onEvent(const KeyboardEvent) override;
    virtual bool onEvent(const MouseEvent) override;

private:
#ifndef EMSCRIPTEN
    TwBar* twBar = nullptr;
#endif
    void checkInit();
};

#endif // TweakBar_Hpp
