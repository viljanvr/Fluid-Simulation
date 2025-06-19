#include "ScenePresets.h"


void ScenePresets::setScene(int N, int scene_id, std::vector<RectangleObstacle *> &obstacles,
                            bool &vorticity_conf_enabled, bool &collision_enabled, bool &pressure_force_enabled,
                            bool &temp_enabled, InteractionMode &interaction_mode) {
    switch (scene_id) {
        case 1:
            vorticity_conf_enabled = false;
            collision_enabled = false;
            pressure_force_enabled = false;
            temp_enabled = false;
            interaction_mode = SOLID;
            break;
        case 2:
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.3f), 0.4f, 0.05f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.6f), 0.1f, 0.10f, 1.0f));

            vorticity_conf_enabled = true;
            collision_enabled = false;
            pressure_force_enabled = false;
            temp_enabled = false;
            interaction_mode = SOLID;
            break;
        case 3:
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.3f), 0.4f, 0.05f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.6f), 0.1f, 0.10f, 1.0f));

            vorticity_conf_enabled = true;
            collision_enabled = true;
            pressure_force_enabled = false;
            temp_enabled = false;
            interaction_mode = RIGID;
            break;

        case 4:
            vorticity_conf_enabled = true;
            collision_enabled = false;
            pressure_force_enabled = false;
            temp_enabled = true;
            interaction_mode = SOLID;
            break;
        case 5:
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.3f), 0.4f, 0.05f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.6f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.2f, 0.9f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.8f, 0.9f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.8f, 0.4f), 0.1f, 0.10f, 1.0f));
            obstacles.push_back(new RectangleObstacle(N, Vec2f(0.3f, 0.4f), 0.1f, 0.10f, 1.0f));

            vorticity_conf_enabled = true;
            collision_enabled = true;
            pressure_force_enabled = true;
            temp_enabled = true;
            interaction_mode = RIGID;
            break;
        default:

            break;
    }
}
