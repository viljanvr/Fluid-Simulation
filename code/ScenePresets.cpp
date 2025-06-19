#include "ScenePresets.h"


void ScenePresets::setScene(int N, int scene_id, std::vector<RectangleObstacle *> &obstacles,
                            bool &vorticity_conf_enabled, bool &collision_enabled, bool &pressure_force_enabled,
                            bool &temp_enabled, InteractionMode &interaction_mode) {
    switch (scene_id) {
        case 1:
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.3f), 0.4f, 0.05f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.6f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.2f, 0.9f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.8f, 0.9f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.8f, 0.4f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.3f, 0.4f), 0.1f, 0.10f, 1.0f));
            break;
        default:

            break;
    }
}
