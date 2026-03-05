#include "machine_viewer.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rerun.hpp>

namespace {

constexpr std::array<const char*, 3> kAxes = {"x", "y", "z"};
constexpr std::array<const char*, 4> kParts = {"ground", "x", "y", "z"};

bool is_axis(const std::string& name) {
  return name == "x" || name == "y" || name == "z";
}

bool is_part(const std::string& name) {
  return name == "ground" || is_axis(name);
}

std::string sanitize_metric_name(std::string name) {
  std::replace(name.begin(), name.end(), ' ', '_');
  return name;
}

rerun::datatypes::Vec3D axis_translation(const std::string& axis, double value) {
  const auto v = static_cast<float>(value);
  if (axis == "x") {
    return rerun::datatypes::Vec3D(v, 0.0F, 0.0F);
  }
  if (axis == "y") {
    return rerun::datatypes::Vec3D(0.0F, v, 0.0F);
  }
  return rerun::datatypes::Vec3D(0.0F, 0.0F, v);
}

}  // namespace

struct MachineViewer::Impl {
  Impl(
    std::filesystem::path models_dir,
    linkage_map_t linkages,
    model_map_t model_files,
    std::string recording_name,
    double tool_z_offset
  )
      : _rec(std::move(recording_name)),
        _models_dir(std::move(models_dir)),
        _linkages(std::move(linkages)),
        _model_files(std::move(model_files)),
        _tool_z_offset(tool_z_offset) {
    validate_configuration();
    build_entity_paths();
    initialize_viewer();
    update_position({0.0, 0.0, 0.0});
  }

  void update_position(const std::array<double, 3>& xyz) {
    ++_tick;
    _rec.set_time_sequence("tick", _tick);

    for (std::size_t i = 0; i < kAxes.size(); ++i) {
      const std::string axis = kAxes[i];
      const auto translation = rerun::components::Translation3D(axis_translation(axis, xyz[i]));
      _rec.log(_entity_paths.at(axis), rerun::Transform3D(translation));
    }

    if (_tool_loaded) {
      const auto tip = rerun::datatypes::Vec3D(
        static_cast<float>(xyz[0]),
        static_cast<float>(xyz[1]),
        static_cast<float>(xyz[2] - _tool_length + _tool_z_offset)
      );
      _tool_tip_trace.emplace_back(tip);

      rerun::components::LineStrip3D strip(_tool_tip_trace);
      _rec.log(
        _root_path + "/tool_trace",
        rerun::LineStrips3D({strip}).with_colors({rerun::Rgba32(255, 165, 0)}).with_radii({0.0025F})
      );
    }
  }

  void log_scalar(const std::string& name, double value) {
    if (name.empty()) {
      throw std::invalid_argument("MachineViewer scalar name cannot be empty.");
    }

    ++_tick;
    _rec.set_time_sequence("tick", _tick);
    _rec.log(_root_path + "/metrics/" + sanitize_metric_name(name), rerun::Scalars(value));
  }

  void load_tool(double length, double diameter) {
    if (length <= 0.0) {
      throw std::invalid_argument("MachineViewer tool length must be > 0.");
    }
    if (diameter <= 0.0) {
      throw std::invalid_argument("MachineViewer tool diameter must be > 0.");
    }

    const auto length_f = static_cast<float>(length);
    const auto radius_f = static_cast<float>(diameter * 0.5);
    const auto center = rerun::components::PoseTranslation3D(
      0.0F,
      0.0F,
      -0.5F * length_f + static_cast<float>(_tool_z_offset)
    );

    const auto tool = rerun::Cylinders3D::from_lengths_and_radii({length_f}, {radius_f})
                        .with_centers({center})
                        .with_colors({rerun::Rgba32(255, 165, 0)});

    const bool geometry_changed = !_tool_loaded ||
                                  std::abs(_tool_length - length) > std::numeric_limits<double>::epsilon() ||
                                  std::abs(_tool_diameter - diameter) > std::numeric_limits<double>::epsilon();

    _rec.log_static(_entity_paths.at(_tool_mount_part) + "/tool", tool);
    _tool_loaded = true;
    _tool_length = length;
    _tool_diameter = diameter;
    if (geometry_changed) {
      _tool_tip_trace.clear();
      _rec.log(_root_path + "/tool_trace", rerun::Clear::FLAT);
    }
  }

  void unload_tool() {
    _tool_loaded = false;
    _tool_length = 0.0;
    _tool_tip_trace.clear();
    _rec.log(_entity_paths.at(_tool_mount_part) + "/tool", rerun::Clear::FLAT);
    _rec.log(_root_path + "/tool_trace", rerun::Clear::FLAT);
  }

  double tool_z_offset() const {
    return _tool_z_offset;
  }

  void set_tool_z_offset(double tool_z_offset) {
    _tool_z_offset = tool_z_offset;
  }

private:
  void validate_configuration() {
    if (_models_dir.empty()) {
      throw std::invalid_argument("MachineViewer models_dir cannot be empty.");
    }

    if (!_models_dir.is_absolute()) {
      _models_dir = std::filesystem::absolute(_models_dir);
    }

    for (const auto& axis : kAxes) {
      if (_linkages.find(axis) == _linkages.end()) {
        throw std::invalid_argument("MachineViewer linkage map is missing axis '" + std::string(axis) + "'.");
      }
    }

    for (const auto& [axis, parent] : _linkages) {
      if (!is_axis(axis)) {
        throw std::invalid_argument(
          "MachineViewer linkage key '" + axis + "' is invalid. Allowed keys are x, y, z."
        );
      }
      if (!is_part(parent)) {
        throw std::invalid_argument(
          "MachineViewer linkage parent '" + parent +
          "' is invalid. Allowed parents are ground, x, y, z."
        );
      }
      if (axis == parent) {
        throw std::invalid_argument("MachineViewer linkage cycle: axis '" + axis + "' cannot parent itself.");
      }
    }

    for (const auto& part : kParts) {
      if (_model_files.find(part) == _model_files.end()) {
        throw std::invalid_argument(
          "MachineViewer model map is missing '" + std::string(part) + "' model filename."
        );
      }
    }

    std::unordered_map<std::string, std::string> parent_of;
    parent_of["ground"] = "";
    for (const auto& axis : kAxes) {
      parent_of[axis] = _linkages.at(axis);
    }

    std::unordered_set<std::string> visiting;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&)> dfs = [&](const std::string& node) {
      if (node == "ground" || visited.count(node) > 0) {
        return;
      }
      if (visiting.count(node) > 0) {
        throw std::invalid_argument("MachineViewer linkage map contains a cycle.");
      }
      visiting.insert(node);
      dfs(parent_of.at(node));
      visiting.erase(node);
      visited.insert(node);
    };

    for (const auto& axis : kAxes) {
      dfs(axis);
    }
  }

  void build_entity_paths() {
    std::unordered_map<std::string, std::string> parent_of;
    parent_of["ground"] = "";
    for (const auto& axis : kAxes) {
      parent_of[axis] = _linkages.at(axis);
    }

    std::function<std::string(const std::string&)> resolve_path = [&](const std::string& part) -> std::string {
      if (const auto it = _entity_paths.find(part); it != _entity_paths.end()) {
        return it->second;
      }
      if (part == "ground") {
        const std::string path = _root_path + "/ground";
        _entity_paths[part] = path;
        return path;
      }

      const std::string parent = parent_of.at(part);
      const std::string path = resolve_path(parent) + "/" + part;
      _entity_paths[part] = path;
      return path;
    };

    for (const auto& part : kParts) {
      resolve_path(part);
    }

    std::unordered_set<std::string> leaf_axes = {"x", "y", "z"};
    for (const auto& [axis, parent] : _linkages) {
      (void)axis;
      if (leaf_axes.count(parent) > 0) {
        leaf_axes.erase(parent);
      }
    }

    if (leaf_axes.size() == 1) {
      _tool_mount_part = *leaf_axes.begin();
    } else {
      _tool_mount_part = "z";
    }
  }

  std::filesystem::path resolve_model_path(const std::string& part) const {
    std::filesystem::path path = _model_files.at(part);
    if (path.is_relative()) {
      path = _models_dir / path;
    }
    return path;
  }

  void initialize_viewer() {
    _rec.spawn().exit_on_failure();
    _rec.log_static(_root_path, rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    for (const auto& part : kParts) {
      const auto model_path = resolve_model_path(part);
      if (!std::filesystem::exists(model_path)) {
        throw std::invalid_argument("MachineViewer model file not found: " + model_path.string());
      }

      const auto asset = rerun::Asset3D::from_file_path(model_path).value_or_throw();
      _rec.log_static(_entity_paths.at(part) + "/model", asset);
    }
  }

  rerun::RecordingStream _rec;
  std::filesystem::path _models_dir;
  linkage_map_t _linkages;
  model_map_t _model_files;
  std::unordered_map<std::string, std::string> _entity_paths;
  std::string _root_path = "machine";
  std::string _tool_mount_part = "z";
  double _tool_z_offset = 0.0;
  bool _tool_loaded = false;
  double _tool_length = 0.0;
  double _tool_diameter = 0.0;
  std::vector<rerun::datatypes::Vec3D> _tool_tip_trace;
  std::int64_t _tick = 0;
};

MachineViewer::MachineViewer(
  std::filesystem::path models_dir,
  linkage_map_t linkages,
  model_map_t model_files,
  std::string recording_name,
  double tool_z_offset
) {
  _impl = std::make_unique<Impl>(
    std::move(models_dir),
    std::move(linkages),
    std::move(model_files),
    std::move(recording_name),
    tool_z_offset
  );
}

MachineViewer::~MachineViewer() = default;
MachineViewer::MachineViewer(MachineViewer&&) noexcept = default;
MachineViewer& MachineViewer::operator=(MachineViewer&&) noexcept = default;

void MachineViewer::update_position(const std::array<double, 3>& xyz) {
  _impl->update_position(xyz);
}

void MachineViewer::load_tool(double length, double diameter) {
  _impl->load_tool(length, diameter);
}

void MachineViewer::unload_tool() {
  _impl->unload_tool();
}

void MachineViewer::log_scalar(const std::string& name, double value) {
  _impl->log_scalar(name, value);
}

double MachineViewer::tool_z_offset() const {
  return _impl->tool_z_offset();
}

void MachineViewer::set_tool_z_offset(double tool_z_offset) {
  _impl->set_tool_z_offset(tool_z_offset);
}
