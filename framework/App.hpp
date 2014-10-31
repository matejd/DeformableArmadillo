#ifndef App_Hpp
#define App_Hpp

#include "Platform.hpp"
#include "Events.hpp"
#include "Renderer.hpp"

class Renderer;

class App: public EventListener
{
public:
    App(int argc, char** argv);
    virtual ~App();

    virtual bool setup() = 0;
    virtual void drawFrame() = 0;

    int exec();

    void addEventListener(EventListener* listener);

protected:
    Renderer renderer;
    int canvasWidth, canvasHeight;
};

#endif // App_Hpp
