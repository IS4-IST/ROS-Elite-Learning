// Shared helpers: model loading, a demo joint controller, image output.
#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <vector>

namespace ec {

struct ModelDeleter { void operator()(mjModel* m) const { mj_deleteModel(m); } };
struct DataDeleter  { void operator()(mjData* d)  const { mj_deleteData(d); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr  = std::unique_ptr<mjData, DataDeleter>;

std::string default_model_path();

ModelPtr load_model(const std::string& path);

void reset_home(const mjModel* m, mjData* d);

struct WiggleController {
    double amplitude = 0.35;   // rad (~20 deg)
    double frequency = 0.25;   // Hz
    std::vector<double> home;  // filled from the keyframe on first use
    void apply(const mjModel* m, mjData* d);
};

bool write_ppm(const std::string& path, const unsigned char* rgb, int width, int height);

} 
