// Headless OpenGL context through EGL, for offscreen rendering without a display.
#pragma once

#include <EGL/egl.h>

namespace ec {

class EglContext {
public:
    EglContext();
    ~EglContext();
    EglContext(const EglContext&) = delete;
    EglContext& operator=(const EglContext&) = delete;

    const char* vendor() const; 
    const char* renderer() const;

private:
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLSurface surface_ = EGL_NO_SURFACE;
    EGLContext context_ = EGL_NO_CONTEXT;
};

}  // namespace ec
