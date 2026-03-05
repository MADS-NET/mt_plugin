#include "machine_viewer.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <thread>

namespace {

std::filesystem::path resolve_models_dir(int argc, char* argv[]) {
  if (argc > 1) {
    return std::filesystem::path(argv[1]);
  }

  const auto cwd = std::filesystem::current_path();
  const auto direct = cwd / "models";
  if (std::filesystem::exists(direct)) {
    return direct;
  }

  const auto parent = cwd / ".." / "models";
  if (std::filesystem::exists(parent)) {
    return std::filesystem::weakly_canonical(parent);
  }

  throw std::runtime_error(
    "Unable to find models directory. Pass it as first argument, e.g. ./machine_viewer_example ../models"
  );
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto models_dir = resolve_models_dir(argc, argv);

    const std::map<std::string, std::string> linkages = {
      {"x", "ground"},
      {"z", "x"},
      {"y", "z"},
    };

    const std::map<std::string, std::string> model_files = {
      {"ground", "ground.obj"},
      {"x", "x_axis.obj"},
      {"y", "y_axis.obj"},
      {"z", "z_axis.obj"},
    };

    MachineViewer viewer(models_dir, linkages, model_files, "machine_viewer_example");

    constexpr int total_steps = 50;
    constexpr std::chrono::milliseconds tick{100};
    for (int i = 0; i <= total_steps; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(total_steps);
      const double position = 0.5 * alpha;
      const std::array<double, 3> xyz = {position, position, position};
      viewer.update_position(xyz);
      viewer.log_scalar("progress", alpha);

      if (i < total_steps) {
        std::this_thread::sleep_for(tick);
      }
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "machine_viewer_example failed: " << ex.what() << '\n';
    return 1;
  }
}
