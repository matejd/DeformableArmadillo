#ifndef OrbitalCamera_Hpp
#define OrbitalCamera_Hpp

#include "Events.hpp"
#include "Platform.hpp"

class OrbitalCamera: public EventListener
{
public:
    int canvasWidth   = 1280;
    int canvasHeight  = 720;
    float cameraPhi   = 0.f;
    float cameraTheta = PI2;
    float cameraR     = 2.f;

    float cameraMaxTheta = PI2;
    float cameraMaxR = 3.f;
    float cameraMinR = 1.f;

    float cameraNear  = 0.1f;
    float cameraFar   = 8.f;
    float fovDegrees  = 77.f;
    Vec3 worldUp      = Vec3(0.f, 1.f, 0.f);
    Point3 cameraTarget = Point3(0.f, 0.f, 0.f);

    const Matrix4 getTransformMatrix();
    const Point3 getPosition();

    virtual bool onEvent(const KeyboardEvent) override;
    virtual bool onEvent(const MouseEvent) override;

private:
    bool rotating = false;
    bool scaling = false;
    int mouseStartX, mouseStartY;
};

#endif // OrbitalCamera_Hpp
