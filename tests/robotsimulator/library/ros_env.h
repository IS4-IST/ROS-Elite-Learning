// Run a ROS 2 program without `source /opt/ros/<distro>/setup.bash`.
//
// At configure time CMake only needs the install's prefix (EC_ROS_DIR). At run time ROS
// dlopen()s its middleware and per-message support libraries, found through
// AMENT_PREFIX_PATH (read when needed) and LD_LIBRARY_PATH (read by the loader ONLY at
// process start). So: set both and restart this same binary once. If the shell already
// sourced ROS, nothing happens.
//
// Call it as the first line of main(), before anything else runs.
#pragma once

#ifdef EC_HAVE_ROS

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>

namespace ec {

inline void ensure_ros_environment(char** argv) {
    if (std::getenv("AMENT_PREFIX_PATH")) return;   // already sourced, or already restarted
    const std::string dir = EC_ROS_DIR;
    setenv("AMENT_PREFIX_PATH", dir.c_str(), 1);
    const char* old = std::getenv("LD_LIBRARY_PATH");
    //const std::string ld = dir + "/lib" + (old && *old ? std::string(":") + old : std::string());
    const std::string ld = dir + "/lib/x86_64-linux-gnu:" + dir + "/lib" +
                           (old && *old ? std::string(":") + old : std::string());
    setenv("LD_LIBRARY_PATH", ld.c_str(), 1);
    // the real path, not /proc/self/exe: otherwise ps/top would show the program as "exe"
    char self[4096];
    const ssize_t n = readlink("/proc/self/exe", self, sizeof self - 1);
    if (n <= 0) return;
    self[n] = '\0';
    execv(self, argv);   // returns only on failure
    std::perror("restarting with the ROS environment");
}

}  // namespace ec

#endif
