#pragma once

#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <string>

class MachineViewer {
public:
  using linkage_map_t = std::map<std::string, std::string>;
  using model_map_t = std::map<std::string, std::string>;

  MachineViewer(
    std::filesystem::path models_dir,
    linkage_map_t linkages,
    model_map_t model_files,
    std::string recording_name = "machine_viewer",
    double tool_z_offset = 0.0
  );

  ~MachineViewer();
  MachineViewer(const MachineViewer&) = delete;
  MachineViewer& operator=(const MachineViewer&) = delete;
  MachineViewer(MachineViewer&&) noexcept;
  MachineViewer& operator=(MachineViewer&&) noexcept;

  void update_position(const std::array<double, 3>& xyz);
  void load_tool(double length, double diameter);
  void unload_tool();
  void log_scalar(const std::string& name, double value);
  double tool_z_offset() const;
  void set_tool_z_offset(double tool_z_offset);

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};
