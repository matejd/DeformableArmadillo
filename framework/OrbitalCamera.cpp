#include "OrbitalCamera.hpp"

#include "external/glm/gtc/matrix_transform.hpp"

using namespace glm;

const Matrix4 OrbitalCamera::getTransformMatrix()
{
    const Point3 cameraPosition = getPosition();
    const Matrix4 view = lookAt(cameraPosition, cameraTarget, worldUp);
    const float ratio = static_cast<float>(canvasWidth) / canvasHeight;
    const Matrix4 projection = perspective(radians(fovDegrees), ratio, cameraNear, cameraFar);
    return projection * view;
}

const Point3 OrbitalCamera::getPosition()
{
    return cameraR * Point3(sin(cameraTheta)*sin(cameraPhi),
                            cos(cameraTheta),
                            sin(cameraTheta)*cos(cameraPhi))
                   + cameraTarget;
}

bool OrbitalCamera::onEvent(const KeyboardEvent)
{
    return false;
}

bool OrbitalCamera::onEvent(const MouseEvent event)
{
    if (event.action == MouseAction::Move && rotating) {
        const float dx = static_cast<float>(event.mouseX-mouseStartX);
        const float dy = static_cast<float>(event.mouseY-mouseStartY);
        mouseStartX = event.mouseX;
        mouseStartY = event.mouseY;
        cameraPhi   -= (dx / canvasWidth)  * TwoPI;
        cameraTheta -= (dy / canvasHeight) * PI;
        cameraTheta = clamp(cameraTheta, 0.001f, min(PI-0.001f, cameraMaxTheta));
    }

    if (event.action == MouseAction::Move && scaling) {
        const float dy = static_cast<float>(event.mouseY-mouseStartY);
        mouseStartY = event.mouseY;
        cameraR = clamp(cameraR - (dy / canvasHeight) * 5.f, cameraMinR, cameraMaxR);
    }

    if (event.action == MouseAction::Press && event.button == MouseButton::Left) {
        mouseStartX = event.mouseX;
        mouseStartY = event.mouseY;
        rotating = true;
    }

    if (event.action == MouseAction::Press && event.button == MouseButton::Middle) {
        mouseStartY = event.mouseY;
        scaling = true;
    }

    if (event.action == MouseAction::Release) {
        rotating = false;
        scaling = false;
    }

    return true;
}
