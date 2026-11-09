#include "egl_context.h"

#include <EGL/eglext.h>
#include <GL/gl.h>

#include <cstdio>
#include <stdexcept>
#include <string>

namespace ec {
namespace {

std::string egl_error(const char* what) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), " (EGL error 0x%04x)", eglGetError());
    return std::string(what) + buf;
}

EGLDisplay open_display() {
    // Preferred: enumerate EGL devices and open the first one (no X server needed)
    auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(
        eglGetProcAddress("eglQueryDevicesEXT"));
    auto get_platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
        eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if (query_devices && get_platform_display) {
        EGLDeviceEXT devices[8];
        EGLint count = 0;
        if (query_devices(8, devices, &count) && count > 0) {
            EGLDisplay d = get_platform_display(EGL_PLATFORM_DEVICE_EXT, devices[0], nullptr);
            if (d != EGL_NO_DISPLAY) return d;
        }
    }
    return eglGetDisplay(EGL_DEFAULT_DISPLAY);
}

}  // namespace

EglContext::EglContext() {
    display_ = open_display();
    if (display_ == EGL_NO_DISPLAY) throw std::runtime_error(egl_error("no EGL display"));
    EGLint major = 0, minor = 0;
    if (!eglInitialize(display_, &major, &minor)) {
        throw std::runtime_error(egl_error("eglInitialize failed"));
    }

    const EGLint config_attribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_NONE};
    EGLConfig config;
    EGLint num_configs = 0;
    if (!eglChooseConfig(display_, config_attribs, &config, 1, &num_configs) || num_configs < 1) {
        throw std::runtime_error(egl_error("no suitable EGL config"));
    }

    // MuJoCo renders into its own offscreen framebuffer; this surface is only
    // needed so the context can be made current on every EGL implementation.
    const EGLint pbuffer_attribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    surface_ = eglCreatePbufferSurface(display_, config, pbuffer_attribs);
    if (surface_ == EGL_NO_SURFACE) throw std::runtime_error(egl_error("eglCreatePbufferSurface failed"));

    // Desktop OpenGL, default (compatibility) profile, which MuJoCo's classic renderer needs
    if (!eglBindAPI(EGL_OPENGL_API)) throw std::runtime_error(egl_error("eglBindAPI failed"));
    context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, nullptr);
    if (context_ == EGL_NO_CONTEXT) throw std::runtime_error(egl_error("eglCreateContext failed"));
    if (!eglMakeCurrent(display_, surface_, surface_, context_)) {
        throw std::runtime_error(egl_error("eglMakeCurrent failed"));
    }
}

EglContext::~EglContext() {
    if (display_ != EGL_NO_DISPLAY) {
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
        if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
        eglTerminate(display_);
    }
}

const char* EglContext::vendor() const {
    return reinterpret_cast<const char*>(glGetString(GL_VENDOR));
}
const char* EglContext::renderer() const {
    return reinterpret_cast<const char*>(glGetString(GL_RENDERER));
}

}  // namespace ec
