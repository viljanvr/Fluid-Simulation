#pragma once

#include <vector>
#include "RectangleObstacle.h"

enum InteractionMode { SOLID, RIGID, Last };

class ScenePresets {
public:
    static void setScene(int N, int scene_id, std::vector<RectangleObstacle *> &obstacles, bool &vorticity_conf_enabled,
                         bool &collision_enabled, bool &pressure_force_enabled, bool &temp_enabled,
                         InteractionMode &interaction_mode);
};
