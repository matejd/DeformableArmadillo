#ifndef Events_Hpp
#define Events_Hpp

enum class Key
{
    S,
    Enter,
    Escape,
    Space
};

enum class KeyAction
{
    Press,
    Release
};

enum class MouseButton
{
    Left,
    Middle,
    Right,
    Unknown
};

enum class MouseAction
{
    Press,
    Release,
    Move,
    Scroll
};

struct KeyboardEvent
{
    Key key;
    KeyAction action;
};

struct MouseEvent
{
    MouseButton button;
    MouseAction action;
    int mouseX, mouseY;
};

class EventListener
{
public:
    virtual bool onEvent(const KeyboardEvent) { return false; }
    virtual bool onEvent(const MouseEvent) { return false; }
};

#endif // Events_Hpp
