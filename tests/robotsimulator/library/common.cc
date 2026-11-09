#include "common.h"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ec {

std::string default_model_path() { return std::string(EC_MODEL_DIR) + "/scene.xml"; }

ModelPtr load_model(const std::string& path) {
    char error[1024] = "";
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        throw std::runtime_error("could not load " + path + ": " + error);
    }
    return ModelPtr(m);
}

void reset_home(const mjModel* m, mjData* d) {
    int key = mj_name2id(m, mjOBJ_KEY, "home");
    if (key >= 0) {
        mj_resetDataKeyframe(m, d, key);
    } else {
        mj_resetData(m, d);
    }
    mj_forward(m, d);
}

void WiggleController::apply(const mjModel* m, mjData* d) {
    if (home.empty()) {
        int key = mj_name2id(m, mjOBJ_KEY, "home");
        home.assign(m->nu, 0.0);
        for (int i = 0; i < m->nu; ++i) {
            home[i] = key >= 0 ? m->key_ctrl[key * m->nu + i] : 0.0;
        }
    }
    const double w = 2.0 * M_PI * frequency;
    for (int i = 0; i < m->nu; ++i) {
        // phase-shift each joint so the motion is not all in step
        double target = home[i] + amplitude * std::sin(w * d->time + 0.7 * i);
        // respect the actuator's control range
        if (m->actuator_ctrllimited[i]) {
            target = std::fmin(std::fmax(target, m->actuator_ctrlrange[2 * i]),
                               m->actuator_ctrlrange[2 * i + 1]);
        }
        d->ctrl[i] = target;
    }
}

bool write_ppm(const std::string& path, const unsigned char* rgb, int width, int height) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", width, height);
    // OpenGL rows are bottom-up; PPM is top-down
    for (int y = height - 1; y >= 0; --y) {
        std::fwrite(rgb + static_cast<size_t>(y) * width * 3, 1, static_cast<size_t>(width) * 3, f);
    }
    std::fclose(f);
    return true;
}

}  // namespace ec
