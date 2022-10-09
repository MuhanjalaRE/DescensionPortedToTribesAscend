#define CAP_ACCELERATION_

#include "DX.h"
#include "ue.h"

#include "imgui.h"
#include "imgui_impl_dx9.h"
#include "imgui_impl_win32.h"

/*
#include "Other/Files/Files.h"
#include "Other/JSON/json.hpp"
#include "Other/Keys/Keys.h"
*/

#include <chrono>
#include <fstream>
#include <iostream>
#include <utility>

//#define USE_SOL
#ifdef USE_SOL
#include <sol/sol.hpp>
#endif

#include "SDK.h"

#include "Hook.h"
#include "log.h"

#include <unordered_map>

#include "detours.h"

// using json = nlohmann::json;
using namespace std;
// using namespace UE_Utilities;

typedef FVector Vector;
typedef FRotator Rotator;
typedef ATrPlayerPawn Character;

class Timer {
   private:
    std::chrono::steady_clock::time_point t_last_tick_time;
    int i_frequency;
    float f_period;

   public:
    void SetFrequency(int frequency) {
        this->i_frequency = frequency;
        this->f_period = 1.0 / frequency;
    }

    Timer(void) {
        Tick();
        SetFrequency(1);
    }

    Timer(int frequency) : Timer() {
        SetFrequency(frequency);
    }

    bool isReady(void) {
        std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();
        float delta = std::chrono::duration<float>(t_now - t_last_tick_time).count();
        if (delta > f_period) {
            // t_last_tick_time = t_now;
            Tick();
            return true;
        }
        return false;
    }

    void Tick(void) {
        t_last_tick_time = std::chrono::steady_clock::now();
    }
};

namespace imgui {

static auto vector_getter = [](void* vec, int idx, const char** out_text) {
    auto& vector = *static_cast<std::vector<std::string>*>(vec);
    if (idx < 0 || idx >= static_cast<int>(vector.size())) {
        return false;
    }
    *out_text = vector.at(idx).c_str();
    return true;
};

namespace visuals {
static enum MarkerStyle { kNone, kDot, kCircle, kFilledSquare, kSquare, kBounds, kFilledBounds, kFilledRectangle, kRectangle };
static const char* marker_labels[] = {"None", "Dot", "Circle", "Filled square", "Square", "Bounds", "Filled bounds"};
// static const char* marker_labels[] = {"Dot", "Circle", "Filled square", "Square", "Filled rectangle", "Rectangle"};

static struct AimbotVisualSettings {
    MarkerStyle marker_style = MarkerStyle::kFilledBounds;
    int marker_size = 6;
    int marker_thickness = 2;
    ImColor marker_colour = {255, 255, 0, 200};
    int marker_size_width = 5;
    int marker_size_height = 5;

    bool scale_by_distance = true;
    int distance_for_scaling = 5000;
    int minimum_marker_size = 3;

} aimbot_visual_settings;

static struct RouteVisualSettings {
    MarkerStyle marker_style = MarkerStyle::kDot;
    int marker_size = 5;
    int marker_thickness = 1;
    ImColor marker_colour = {255, 0, 0, 0.66 * 255};
    bool rainbow_gradient = true;
    int marker_size_width = 5;
    int marker_size_height = 5;
} route_visual_settings;

static struct RadarVisualSettings {
    MarkerStyle marker_style = MarkerStyle::kDot;
    int marker_size = 3;
    int marker_thickness = 1;
    ImColor enemy_marker_colour = {255, 0, 0, 1 * 255};
    ImColor friendly_marker_colour = {0, 255, 0, 1 * 255};
    ImColor enemy_flag_marker_colour = {255, 255, 0, 255};
    ImColor friendly_flag_marker_colour = {0, 255, 255, 255};
    ImColor window_background_colour = {25, 25, 25, 200};
    bool draw_axes = true;
    // bool draw_lines = false;

    float zoom_ = 0.004;
    float zoom = 1;
    // ImVec2 window_size = {400, 400};
    int window_size = 300;
    ImVec2 window_location = {100, 100};
    float border = 5;
    int axes_thickness = 1;
} radar_visual_settings;

static struct ESPVisualSettings {
    struct BoundingBoxSettings {
        // MarkerStyle marker_style = MarkerStyle::kRectangle;
        int box_thickness = 1;
        ImColor enemy_player_box_colour = {0, 255, 255, 1 * 200};
        ImColor friendly_player_box_colour = {0, 255, 0, 1 * 255};
    } bounding_box_settings;

    struct LineSettings {
        int line_thickness = 1;
        ImColor enemy_player_line_colour = {255, 0, 0, 255};
    } line_settings;

    struct NameSettings {
        int name_height_offset = 30;
        int scale = 1;
        ImColor enemy_name_colour = {255, 255, 0, 1 * 255};
        ImColor friendly_name_colour = {0, 255, 0, 1 * 255};
    } name_settings;

} esp_visual_settings;

static struct CrosshairSettings {
    bool show_or_enabled = true;
    MarkerStyle marker_style = MarkerStyle::kDot;
    int marker_size = 3;
    int marker_thickness = 1;
    ImColor marker_colour = {0, 255, 0, 255};
    int marker_size_width = 5;
    int marker_size_height = 5;
} crosshair_settings;

}  // namespace visuals

static struct ImGuiSettings { } imgui_settings; }  // namespace imgui

namespace validate {
bool IsValid(APawn* pawn) {
    return (pawn && pawn->PlayerReplicationInfo);
}

bool IsValid(Character* character) {
    // https://stackoverflow.com/a/8234993
    try {
        return (character && character->PlayerReplicationInfo && character->Health > 0);
    } catch (...) {
        return false;
    }
}

}  // namespace validate

namespace math {
#define M_PI 3.14159265358979323846
#define M_PI_F ((float)(M_PI))
#define PI M_PI
#define DEG2RAD(x) ((float)(x) * (float)(M_PI_F / 180.f))
#define RAD2DEG(x) ((float)(x) * (float)(180.f / M_PI_F))
#define INV_PI (0.31830988618f)
#define HALF_PI (1.57079632679f)
#define Const_RadToUnrRot 10430.3783504704527

void PrintVector(FVector f, const char* str) {
    cout << str << " : {" << f.X << ", " << f.Y << ", " << f.Z << "}" << endl;
}
void PrintRotator(FRotator f, const char* str) {
    cout << str << " : {" << f.Pitch << ", " << f.Yaw << ", " << f.Roll << "}" << endl;
}

void SinCos(float* ScalarSin, float* ScalarCos, float Value) {
    // Map Value to y in [-pi,pi], x = 2*pi*quotient + remainder.
    float quotient = (INV_PI * 0.5f) * Value;
    if (Value >= 0.0f) {
        quotient = (float)((int)(quotient + 0.5f));
    } else {
        quotient = (float)((int)(quotient - 0.5f));
    }
    float y = Value - (2.0f * PI) * quotient;

    // Map y to [-pi/2,pi/2] with sin(y) = sin(Value).
    float sign;
    if (y > HALF_PI) {
        y = PI - y;
        sign = -1.0f;
    } else if (y < -HALF_PI) {
        y = -PI - y;
        sign = -1.0f;
    } else {
        sign = +1.0f;
    }

    float y2 = y * y;

    // 11-degree minimax approximation
    *ScalarSin = (((((-2.3889859e-08f * y2 + 2.7525562e-06f) * y2 - 0.00019840874f) * y2 + 0.0083333310f) * y2 - 0.16666667f) * y2 + 1.0f) * y;

    // 10-degree minimax approximation
    float p = ((((-2.6051615e-07f * y2 + 2.4760495e-05f) * y2 - 0.0013888378f) * y2 + 0.041666638f) * y2 - 0.5f) * y2 + 1.0f;
    *ScalarCos = sign * p;
}

FVector RotatorToVector_(FRotator rotation) {
    float CP, SP, CY, SY;
    SinCos(&SP, &CP, DEG2RAD(rotation.Pitch));
    SinCos(&SY, &CY, DEG2RAD(rotation.Yaw));
    FVector V = FVector(CP * CY, CP * SY, SP);

    return V;
}

FVector RotatorToVector(const FRotator& R) {
    FVector Vec;
    float fYaw = R.Yaw * (1 / Const_RadToUnrRot);
    float fPitch = R.Pitch * (1 / Const_RadToUnrRot);
    float CosPitch = cos(fPitch);
    Vec.X = cos(fYaw) * CosPitch;
    Vec.Y = sin(fYaw) * CosPitch;
    Vec.Z = sin(fPitch);

    return Vec;
}

FRotator VectorToRotator(FVector v) {
    FRotator rotator;

    rotator.Yaw = atan2(v.Y, v.X) * Const_RadToUnrRot;
    rotator.Pitch = atan2(v.Z, sqrt((v.X * v.X) + (v.Y * v.Y))) * Const_RadToUnrRot;
    rotator.Roll = 0;  // No roll

    return rotator;
}

FRotator VectorToRotator_(FVector v) {
    FRotator rotator;
    rotator.Yaw = RAD2DEG(atan2(v.Y, v.X));
    rotator.Pitch = RAD2DEG(atan2(v.Z, sqrt((v.X * v.X) + (v.Y * v.Y))));
    rotator.Roll = 0;  // No roll

    return rotator;
}

FVector CrossProduct(FVector U, FVector V) {
    return FVector(U.Y * V.Z - U.Z * V.Y, U.Z * V.X - U.X * V.Z, U.X * V.Y - U.Y * V.X);
}

bool IsVectorToRight(FVector base_vector, FVector vector_to_check) {
    FVector right = CrossProduct(base_vector, {0, 0, 1});

    if (right.Dot(vector_to_check) < 0) {
        return true;
    } else {
        return false;
    }
}

float AngleBetweenVector(FVector a, FVector b) {
    // a.b = |a||b|cosx
    a.Z = 0;
    b.Z = 0;

    float dot = a.Dot(b);
    float denom = a.Magnitude() * b.Magnitude();
    float div = dot / denom;
    return acos(div);
}

}  // namespace math

namespace abstraction {

class WorldObject {
   private:
    Vector location_;
    Rotator rotation_;
    Vector velocity_;
    Vector forward_vector_;
    Vector acceleration_;

   public:
    Character* character_ = NULL;

    WorldObject(void) {
        location_ = Vector();
        rotation_ = Rotator();
        velocity_ = Vector();
        forward_vector_ = Vector();
        acceleration_ = Vector();
    }

    WorldObject(Vector location, Rotator rotation) : WorldObject() {
        location_ = location;
        rotation_ = rotation;
    }

    WorldObject(Vector location, Rotator rotation, Vector velocity) : WorldObject(location, rotation) {
        velocity_ = velocity;
    }

    void SetLocation(Vector location) {
        location_ = location;
    }

    void SetRotation(Rotator rotation) {
        rotation_ = rotation;
    }

    void SetVelocity(Vector velocity) {
        velocity_ = velocity;
    }

    void SetForwardVector(Vector forward_vector) {
        forward_vector_ = forward_vector;
    }

    Vector GetLocation(void) {
        return location_;
    }

    Rotator GetRotation(void) {
        return rotation_;
    }

    Vector GetVelocity(void) {
        return velocity_;
    }

    Vector GetAcceleration(void) {
        return acceleration_;
    }

    Vector PredictFutureLocation(float time_in_ms) {
        return location_ + velocity_ * (time_in_ms / 1000.0);
    }

    Vector GetForwardVector(void) {
        return this->forward_vector_;
    }
};

static class WeaponObject {
   public:
    enum WeaponType { kHitscan, kProjectileLinear, kProjectileArching };

    static struct WeaponParameters {
        float bullet_speed_;
        float inheritence_;
        float ping_;  // in ms
        float self_compensation_ping_ = 0;
        bool use_inheritance = true;
        bool use_acceleration = false;
    };

    static struct WeaponAimbotParameters {
        int maximum_iterations = 2 * 10;
        float epsilon = 0.05 / 3;
    };

   private:
    WorldObject* owner_;
    WeaponType weapon_type_ = kProjectileLinear;
    WeaponParameters weapon_parameters_;
    WeaponAimbotParameters aimbot_parameters_;

   public:
    void SetWeaponType(WeaponType weapon_type) {
        weapon_type_ = weapon_type;
    }

    void SetOwner(WorldObject* owner) {
        owner_ = owner;
    }

    WorldObject* GetOwner(void) {
        return owner_;
    }

    WeaponAimbotParameters* GetAimbotParameters(void) {
        return &aimbot_parameters_;
    }

    WeaponParameters* GetWeaponParameters(void) {
        return &weapon_parameters_;
    }

    bool PredictAimAtTarget(WorldObject* target_object, Vector* output_vector = NULL, Vector offset = Vector());
    bool PredictAimAtTarget_(WorldObject* target_object, Vector* output_vector = NULL, Vector offset = Vector());
    bool PredictAimAtTarget_DicksuckingLord(WorldObject* target_object, Vector* output_vector = NULL, Vector offset = Vector());
    Vector FactorInGravity(Vector target_location, Vector target_velocity, float gravity, float time, Vector offset = Vector());
} my_weapon_object;

}  // namespace abstraction

namespace game_data {
static ATrPlayerController* local_player_controller = NULL;
static Character* local_player_character = NULL;

static FVector2D screen_size = FVector2D();
static FVector2D screen_center = FVector2D();

enum class Weapon { none, found, unknown };

namespace information {
class GameActor {
   public:
    int team_id_ = -1;
    FVector location_;

    bool IsSameTeam(GameActor* actor) {
        return this->team_id_ == actor->team_id_;
    }
};

class Player : public GameActor {
   private:
   public:
    Character* character_ = NULL;
    int player_id_;
    // int team_id_ = -1;
    float health_;
    float energy_;
    // FVector location_;
    FRotator rotation_;
    FVector velocity_;
    FVector forward_vector_;
    FVector acceleration_;
    unsigned int ticks = 0;
    bool is_valid_ = false;
    Weapon weapon_ = Weapon::none;
    bool just_respawned = false;
    wstring name_w_;
    string name_c_;

    Player(void) {
        ;
    }

    Player(Character* character) {
        if (!character || !validate::IsValid(character))
            return;

        Setup(character);
    }

    void Setup(Character* character) {
        character_ = character;
        player_id_ = character->PlayerReplicationInfo->UniqueId.Uid.A;
        team_id_ = character->GetTeamNum();

        location_ = character->Location;
        rotation_ = character_->Rotation;
        velocity_ = character_->Velocity;
        forward_vector_ = math::RotatorToVector(rotation_).Unit();

        health_ = character_->Health;
        energy_ = character_->m_fCurrentPowerPool;

        is_valid_ = true;

        name_w_ = wstring(character->PlayerReplicationInfo->PlayerName.Data);
        name_c_ = string(name_w_.begin(), name_w_.end());
    }
};

}  // namespace information

static class GameData {
   public:
    information::Player my_player_information;
    vector<information::Player> players;

    GameData(void) {
        ;
    }

    void Reset(void) {
        players.clear();
        my_player_information.weapon_ = Weapon::none;
        abstraction::my_weapon_object.SetWeaponType(abstraction::WeaponObject::WeaponType::kHitscan);
        my_player_information.is_valid_ = false;  // Invalidate the player every frame
    }

} game_data;
static game_data::information::Player& my_player = game_data::game_data.my_player_information;

void GetPlayers(void) {
    if (!validate::IsValid(local_player_character) && false) {
        return;
    }

    bool my_player_character_found = false;

    // APawn* pawns = local_player_controller->Pawn->WorldInfo->PawnList;
    APawn* pawns = local_player_controller->WorldInfo->PawnList;
    APawn* pawn = pawns;

    while (pawn != NULL) {
        if (validate::IsValid(pawn)) {
            Character* character = (Character*)pawn;
            game_data.players.push_back(information::Player(character));
            if (pawn == local_player_character) {
                my_player.Setup(character);
                my_player_character_found = true;

                /*
                static FVector camera_position;
                static FRotator camera_rotation;
                static FVector camera_direction;

                game_data::local_player_controller->eventGetActorEyesViewPoint(&camera_position, &camera_rotation);
                camera_direction = math::RotatorToVector(camera_rotation).Unit();

                my_player.rotation_ = camera_rotation;
                my_player.forward_vector_ = camera_direction;
                */
            }
        }

        if (pawn != NULL || pawn->PlayerReplicationInfo != NULL)
            pawn = pawn->NextPawn;
    }

    if (!my_player_character_found) {
        my_player.rotation_ = local_player_controller->Rotation;
        my_player.forward_vector_ = math::RotatorToVector(my_player.rotation_);
        // return;
    }
}

void GetWeapon(void);

void GetGameData(void) {
    game_data.Reset();
    GetPlayers();
    GetWeapon();
}

}  // namespace game_data

namespace aimbot {
/* static */ float delta_time = 0;
vector<game_data::information::Player> players_previous;
/* static */ std::chrono::steady_clock::time_point previous_tick = std::chrono::steady_clock::now();

enum AimbotMode { kClosestDistance, kClosestXhair };
static const char* mode_labels[] = {"Closest distance", "Closest to xhair"};
static bool enabled = true;
extern bool aimbot_enabled = false;

static struct AimbotSettings {
    AimbotMode aimbot_mode = AimbotMode::kClosestXhair;

    bool enabled = true;  // enabling really just enables aimassist, this isnt really an aimbot
    bool use_custom_ping = false;
    bool auto_aim = false;         // this enables the aimbot
    bool target_everyone = false;  // if we want to do prediction on every single player

    float ping_in_ms = 0;  //-90
    float unknowndata00[4];

    int maximum_iterations = 10;
    int epsilon = 0.05;

    bool stay_locked_to_target = true;
    bool auto_lock_to_new_target = false;

    float aimbot_horizontal_fov_angle = 89;         // 30;
    float aimbot_horizontal_fov_angle_cos = 0;      // 0.86602540378;
    float aimbot_horizontal_fov_angle_cos_sqr = 0;  // 0.75;

    bool friendly_fire = false;
    bool need_line_of_sight = false;

    int aimbot_poll_frequency = 60;

    bool use_triggerbot = false;
    float triggerbot_pixel_radius = 15;

    float self_compensation_time_in_ms = 0;

    FVector aimbot_offset = {0, 0, 50};

    bool use_weighting = false;
    int client_weight = 0;
    int prediction_weight = 1;

    float aimbot_vertical_fov_angle = 30;
    float aimbot_vertical_fov_angle_sin = 0.5;
    float aimbot_vertical_fov_angle_sin_sqr = 0.25;

    bool use_acceleration = true;
    float acceleration_cap_x = 2000;
    float acceleration_cap_y = 2000;
    float acceleration_cap_z = 2000;
    bool aimbot_target_center_of_body = true;

} aimbot_settings;
}  // namespace aimbot

namespace abstraction {
bool WeaponObject::PredictAimAtTarget_(WorldObject* target_object, Vector* output_vector, Vector offset) {
    if (!owner_) {
        return false;
    }

    if (weapon_type_ == WeaponType::kHitscan) {
        *output_vector = target_object->GetLocation();
        return true;
    }

    if (weapon_type_ == WeaponType::kProjectileArching) {
        return false;
    }

    /*
    if (weapon_type_ == WeaponType::kProjectileArching) {
                    return PredictAimAtTarget_DicksuckingLord(target_object, output_vector, offset);
    }
    */

    Vector owner_location = owner_->GetLocation() + offset;
    Vector owner_velocity = owner_->GetVelocity();

    owner_location = owner_location - owner_velocity * (weapon_parameters_.self_compensation_ping_ / 1000.0);

    Vector target_location = target_object->GetLocation();
    Vector target_velocity = target_object->GetVelocity();
    Vector target_acceleration = target_object->GetAcceleration();

    if (aimbot::aimbot_settings.use_acceleration) {
        FVector velocity_previous = FVector();
        bool player_found = false;
        for (vector<game_data::information::Player>::iterator player = aimbot::players_previous.begin(); player != aimbot::players_previous.end(); player++) {
            if (player->character_ == target_object->character_) {
                player_found = true;
                velocity_previous = player->velocity_;
                break;
            }
        }

        if (player_found) {
            target_acceleration = (target_velocity - velocity_previous) / (aimbot::delta_time / 1000.0);
        }
    }

    float ping_time = weapon_parameters_.ping_ / 1000.0;

    Vector prediction = target_location;
    Vector ping_prediction = target_location;

    static vector<double> D(aimbot_parameters_.maximum_iterations, 0);
    static vector<double> dt(aimbot_parameters_.maximum_iterations, 0);

    int i = 0;
    do {
        D[i] = (owner_location - prediction).Magnitude();
        dt[i] = D[i] / weapon_parameters_.bullet_speed_;
        if (i > 0 && abs(dt[i] - dt[i - 1]) < aimbot_parameters_.epsilon) {
            break;
        }

        prediction = (target_location + (target_velocity * dt[i] * 1) + (target_acceleration * pow(dt[i], 2) * 0.5) - (weapon_parameters_.use_inheritance ? (owner_velocity * (weapon_parameters_.inheritence_ * dt[i])) : Vector()));
        i++;
    } while (i < aimbot_parameters_.maximum_iterations);

    if (i == aimbot_parameters_.maximum_iterations) {
        return false;
    }

    float full_time = dt[i] + ping_time;

    ping_prediction = prediction = (target_location + (target_velocity * full_time * 1) + (target_acceleration * pow(full_time, 2) * 0.5) - (weapon_parameters_.use_inheritance ? (owner_velocity * (weapon_parameters_.inheritence_ * full_time)) : Vector()));

    *output_vector = ping_prediction;

    if (weapon_type_ == WeaponType::kProjectileArching) {
        // ping_prediction = FactorInGravity(target_location, target_velocity, 1500, full_time, offset);
        return false;
        *output_vector = ping_prediction;
    }

    return true;
}

bool WeaponObject::PredictAimAtTarget(WorldObject* target_object, Vector* output_vector, Vector offset) {
    if (!owner_) {
        return false;
    }

    if (weapon_type_ == WeaponType::kHitscan) {
        *output_vector = target_object->GetLocation();
        return true;
    }

    if (weapon_type_ == WeaponType::kProjectileArching) {
        return false;
    }

    /*
    if (weapon_type_ == WeaponType::kProjectileArching) {
                    return PredictAimAtTarget_DicksuckingLord(target_object, output_vector, offset);
    }
    */

    Vector owner_location = owner_->GetLocation() + offset;
    Vector owner_velocity = owner_->GetVelocity();

    owner_location = owner_location - owner_velocity * (weapon_parameters_.self_compensation_ping_ / 1000.0);

    Vector target_location = target_object->GetLocation();
    Vector target_velocity = target_object->GetVelocity();
    Vector target_acceleration = target_object->GetAcceleration();

    if (aimbot::aimbot_settings.use_acceleration) {
        FVector velocity_previous = FVector();
        bool player_found = false;
        for (vector<game_data::information::Player>::iterator player = aimbot::players_previous.begin(); player != aimbot::players_previous.end(); player++) {
            if (player->character_ == target_object->character_) {
                player_found = true;
                velocity_previous = player->velocity_;
                break;
            }
        }

        if (player_found) {
            target_acceleration = (target_velocity - velocity_previous) / (aimbot::delta_time / 1000.0);

#ifdef CAP_ACCELERATION
            target_acceleration.X = abs(target_acceleration.X) > aimbot::aimbot_settings.acceleration_cap_x ? copysign(aimbot::aimbot_settings.acceleration_cap_x, target_acceleration.X) : target_acceleration.X;
            target_acceleration.Y = abs(target_acceleration.Y) > aimbot::aimbot_settings.acceleration_cap_y ? copysign(aimbot::aimbot_settings.acceleration_cap_y, target_acceleration.Y) : target_acceleration.Y;
            target_acceleration.Z = abs(target_acceleration.Z) > aimbot::aimbot_settings.acceleration_cap_z ? copysign(aimbot::aimbot_settings.acceleration_cap_z, target_acceleration.Z) : target_acceleration.Z;
#endif
            // cout << target_acceleration.X << ", " << target_acceleration.Y << ", " << target_acceleration.Z << endl;
        }
    }

    float ping_time = weapon_parameters_.ping_ / 1000.0;

    Vector target_ping_prediction = target_location + (target_velocity * ping_time * 1) + (target_acceleration * pow(ping_time, 2) * 0.5);
    Vector prediction = target_ping_prediction;
    Vector ping_prediction = target_ping_prediction;

    static vector<double> D(aimbot_parameters_.maximum_iterations, 0);
    static vector<double> dt(aimbot_parameters_.maximum_iterations, 0);

    int i = 0;
    do {
        D[i] = (owner_location - prediction).Magnitude();
        dt[i] = D[i] / weapon_parameters_.bullet_speed_;
        if (i > 0 && abs(dt[i] - dt[i - 1]) < aimbot_parameters_.epsilon) {
            break;
        }

        prediction = (target_ping_prediction + (target_velocity * dt[i] * 1) + (target_acceleration * pow(dt[i], 2) * 0.5) - (weapon_parameters_.use_inheritance ? (owner_velocity * (weapon_parameters_.inheritence_ * dt[i])) : Vector()));
        i++;
    } while (i < aimbot_parameters_.maximum_iterations);

    if (i == aimbot_parameters_.maximum_iterations) {
        return false;
    }

    ping_prediction = prediction = (target_ping_prediction + (target_velocity * dt[i] * 1) + (target_acceleration * pow(dt[i], 2) * 0.5) - (weapon_parameters_.use_inheritance ? (owner_velocity * (weapon_parameters_.inheritence_ * dt[i])) : Vector()));

    *output_vector = ping_prediction;

    if (weapon_type_ == WeaponType::kProjectileArching) {
        // ping_prediction = FactorInGravity(target_location, target_velocity, 1500, full_time, offset);
        return false;
        *output_vector = ping_prediction;
    }

    return true;
}
}  // namespace abstraction

namespace game_functions {
bool InLineOfSight(AActor* actor) {
    return game_data::local_player_character->LineOfSightTo(actor);
}

FVector2D Project(FVector location) {
    FVector projection_3d = game_data::local_player_controller->myHUD->Canvas->Project(location);
    return {projection_3d.X, projection_3d.Y};
    // return {0, 0};
}

FVector2D Project_UsingCharacter(FVector location) {
    FVector2D projection;
    if (game_data::local_player_character) {
        ATrHUD* hud = game_data::local_player_character->GetTrHud();
        if (hud->TrPlayerOwner) {
            FVector projection_3d = game_data::local_player_character->GetTrHud()->Canvas->Project(location);
            projection = {projection_3d.X, projection_3d.Y};
            return projection;
        }
    }
    return {0, 0};
}

bool IsInFieldOfView(FVector enemy_location) {
    FVector rotation_vector = game_data::my_player.forward_vector_;
    FVector difference_vector = enemy_location - game_data::my_player.location_;
    double dot = rotation_vector.X * difference_vector.X + rotation_vector.Y * difference_vector.Y;
    if (dot <= 0)      // dot is > 0 if vectors face same direction, 0 if vectors perpendicular, negative if facing oppposite directions
        return false;  // we want to ensure the vectors face in the same direction
    return true;
}

bool IsInHorizontalFieldOfView(FVector enemy_location, double horizontal_fov) {
    // using namespace math;
    FVector rotation_vector = game_data::my_player.forward_vector_;
    FVector difference_vector = enemy_location - game_data::my_player.location_;
    double dot = rotation_vector.X * difference_vector.X + rotation_vector.Y * difference_vector.Y;
    if (dot <= 0)      // dot is > 0 if vectors face same direction, 0 if vectors perpendicular, negative if facing oppposite directions
        return false;  // we want to ensure the vectors face in the same direction

    double dot_sq = dot * dot;  // squaring the dot loses the negative sign, so we cant tell from this point onwards
                                // if the enemy is in front or behind us if using dot_sq
    double denom = (rotation_vector.X * rotation_vector.X + rotation_vector.Y * rotation_vector.Y) * (difference_vector.X * difference_vector.X + difference_vector.Y * difference_vector.Y);
    double angle_sq = dot_sq / denom;
    double v = pow(cos(DEG2RAD(horizontal_fov)), 2);

    if (angle_sq < v)
        return false;
    return true;
}

}  // namespace game_functions

namespace other {
static struct OtherSettings { bool instant_respawn = true; } other_settings;

void SendLeftMouseClick(void);

void SendLeftMouseClick(void) {
    INPUT inputs[2];
    ZeroMemory(inputs, sizeof(inputs));

    inputs[0].type = INPUT_MOUSE;
    inputs[0].mi.dwFlags = MOUSEEVENTF_LEFTDOWN;

    inputs[1].type = INPUT_MOUSE;
    inputs[1].mi.dwFlags = MOUSEEVENTF_LEFTUP;

    UINT uSent = SendInput(ARRAYSIZE(inputs), inputs, sizeof(INPUT));
}

void Tick(void) {
    if (other_settings.instant_respawn && !game_data::my_player.is_valid_) {
        game_data::local_player_controller->RequestRespawn();
        game_data::local_player_controller->Respawn();
        game_data::local_player_controller->ServerRequestRespawn();
    }
}

}  // namespace other

namespace esp {

static struct ESPSettings {
    bool enabled = true;
    int poll_frequency = 60;
    bool show_friendlies = false;
    int player_height = 100;
    int player_width = 0;
    float width_to_height_ratio = 0.5;
    bool show_lines = true;
    bool show_names = true;

} esp_settings;

static Timer get_esp_data_timer(esp_settings.poll_frequency);

struct ESPInformation {
    FVector2D projection;  // center
    float height;          // height for box/rectangle
    float width;           // width for box/rectangle
    bool is_friendly = false;
    string name;
};

vector<ESPInformation> esp_information;

void Tick(void) {
    if (!esp_settings.enabled || !get_esp_data_timer.isReady())
        return;

    esp_information.clear();

    if (!game_data::my_player.is_valid_)
        ;  // return;

    // bool is_my_player_alive = game_data::my_player.is_valid_;
    for (vector<game_data::information::Player>::iterator player = game_data::game_data.players.begin(); player != game_data::game_data.players.end(); player++) {
        if (!player->is_valid_ || player->character_ == game_data::my_player.character_ || !validate::IsValid(player->character_)) {
            continue;
        }

        game_data::information::Player* p = (game_data::information::Player*)&*player;
        bool same_team = game_data::my_player.IsSameTeam(p);
        // bool same_team = game_functions::IsSameTeam(game_data::local_player_character, player->character_);
        // bool line_of_sight = game_functions::InLineOfSight(player->character_);

        if ((same_team && !esp_settings.show_friendlies) || !game_functions::IsInFieldOfView(player->location_))
            continue;

        if (!game_functions::IsInFieldOfView(player->location_))
            continue;

        FVector2D center_projection = game_functions::Project(player->location_);
        player->location_.Z += player->character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
        FVector2D head_projection = game_functions::Project(player->location_);
        player->location_.Z -= player->character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
        float height = abs(head_projection.Y - center_projection.Y);

        string name;
        if (esp_settings.show_names) {
            name = player->name_c_;
        }

        esp_information.push_back({center_projection, height, height * (player->character_->CylinderComponent->CollisionRadius / player->character_->CylinderComponent->CollisionHeight), same_team, name});
    }
}
}  // namespace esp

namespace aimbot {

// Overshooting means the weapon bullet speed is too low
// Undershooting means the weapon bullet speed is too high

static Timer aimbot_poll_timer(aimbot_settings.aimbot_poll_frequency);

static game_data::information::Player target_player;

vector<FVector2D> projections_of_predictions;

struct AimbotInformation {
    float distance_;
    FVector2D projection_;
    float height;
    float width;
};

vector<AimbotInformation> aimbot_information;

// ImColor colours[] = {ImColor(0, 113, 188, 175), ImColor(0, 127, 0, 175), ImColor(216, 82, 24, 175), ImColor(0, 0, 255, 175), ImColor(236, 176, 31, 175), ImColor(255, 0, 0, 175), ImColor(125, 46, 141, 175), ImColor(0, 191, 191, 175), ImColor(118, 171, 47, 175), ImColor(191, 0, 191, 175), ImColor(76, 189, 237, 175), ImColor(191, 191, 0, 175), ImColor(161, 19, 46, 175), ImColor(63, 63, 63, 175)};
ImColor colours[] = {ImColor(255, 0, 0, 255), ImColor(0, 255, 0, 255), ImColor(0, 0, 255, 255)};

vector<pair<FVector2D, ImColor>> projections_of_predictions_coloured;

void Enable(void) {
    enabled = true;
}

void Reset(void) {
    target_player = game_data::information::Player((Character*)NULL);
    Enable();
}

void Disable(void) {
    Reset();
    enabled = false;
}

void Toggle(void) {
    aimbot_settings.enabled = !aimbot_settings.enabled;
    if (!aimbot_settings.enabled) {
        Reset();
    }
    return;

    if (enabled) {
        Disable();
    } else {
        Enable();
    }
}

bool FindTarget(void) {
    Character* current_target_character = target_player.character_;

    bool need_to_find_target = true;  //! aimbot_settings.target_everyone;

    if (aimbot_settings.stay_locked_to_target) {
        bool find_if_current_target_exists_in_players_list = false;
        for (vector<game_data::information::Player>::iterator player = game_data::game_data.players.begin(); player != game_data::game_data.players.end(); player++) {
            if (player->character_ == target_player.character_) {
                find_if_current_target_exists_in_players_list = true;
                break;
            }
        }

        if (!find_if_current_target_exists_in_players_list || !validate::IsValid(target_player.character_)) {
            if (!aimbot_settings.auto_lock_to_new_target && current_target_character != NULL) {
                Disable();
                return false;
            }
        } else {
            need_to_find_target = false;
        }
    }

    if (need_to_find_target) {
        current_target_character = NULL;
        FVector rotation_vector = game_data::my_player.forward_vector_;
        for (vector<game_data::information::Player>::iterator player = game_data::game_data.players.begin(); player != game_data::game_data.players.end(); player++) {
            if (!player->is_valid_ || player->character_ == game_data::my_player.character_ || !validate::IsValid(player->character_)) {
                continue;
            }

            game_data::information::Player* p = (game_data::information::Player*)&*player;
            bool same_team = game_data::my_player.IsSameTeam(p);  // game_functions::IsSameTeam(game_data::local_player_character, player->character_);
            bool line_of_sight = game_functions::InLineOfSight(player->character_);
            if ((same_team && !aimbot_settings.friendly_fire) || (!game_functions::IsInFieldOfView(player->location_) && aimbot_settings.aimbot_mode == AimbotMode::kClosestXhair) || (!line_of_sight && aimbot_settings.need_line_of_sight))
                continue;

            switch (aimbot_settings.aimbot_mode) {
                case AimbotMode::kClosestXhair: {
                    static double distance = 0;
                    FVector enemy_location = player->location_;

                    if (!game_functions::IsInHorizontalFieldOfView(player->location_, aimbot_settings.aimbot_horizontal_fov_angle))
                        continue;

                    FVector2D projection_vector = game_functions::Project(enemy_location);
                    if (!current_target_character) {
                        current_target_character = player->character_;
                        distance = (projection_vector - game_data::screen_center).MagnitudeSqr();
                    } else {
                        double current_distance = (projection_vector - game_data::screen_center).MagnitudeSqr();
                        if (current_distance < distance) {
                            current_target_character = player->character_;
                            distance = current_distance;
                        }
                    }
                } break;

                case AimbotMode::kClosestDistance: {
                    static double distance = 0;
                    if (!current_target_character) {
                        current_target_character = player->character_;
                        distance = (game_data::my_player.location_ - player->location_).MagnitudeSqr();
                    } else {
                        double current_distance = (game_data::my_player.location_ - player->location_).MagnitudeSqr();
                        if (current_distance < distance) {
                            current_target_character = player->character_;
                            distance = current_distance;
                        }
                    }
                } break;
            }
        }

        if (!current_target_character && !aimbot_settings.auto_lock_to_new_target) {
            // Disable();
            // return false;
        }
    }

    target_player = game_data::information::Player(current_target_character);

    // return true;
    return current_target_character != NULL;
}

static float xy_velocity_scalar = 0;  // 15
static float z_velocity_scalar = 0;   // 5? 0.01

static bool use_muzzle_location = true;
static float muzzle_scalar = 1;

static int fire_mode = 0;

void Tick(void) {
    if (!aimbot_settings.enabled /*|| !enabled*/ || !aimbot_poll_timer.isReady())
        return;

    // SetupWeapon();

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    delta_time = std::chrono::duration<float>(now - previous_tick).count() * 1000.0;
    previous_tick = now;

    projections_of_predictions.clear();
    projections_of_predictions_coloured.clear();
    aimbot_information.clear();

    if (!game_data::my_player.is_valid_ || game_data::my_player.weapon_ == game_data::Weapon::none || game_data::my_player.weapon_ == game_data::Weapon::unknown)
        return;

    static abstraction::WorldObject my_player_world_object;
    my_player_world_object.SetLocation(game_data::my_player.location_);
    my_player_world_object.SetVelocity(game_data::my_player.velocity_);
    abstraction::my_weapon_object.SetOwner(&my_player_world_object);

    static abstraction::WorldObject player_world_object;
    static FVector prediction;
    FVector muzzle_offset;

    if (use_muzzle_location) {
        // muzzle_offset = game_data::my_player.character_->Weapon->FireOffset;
        muzzle_offset = game_data::my_player.character_->eventGetWeaponStartTraceLocation(game_data::my_player.character_->Weapon) - game_data::my_player.location_;
    }

    if (!aimbot_settings.target_everyone) {
        bool triggerbot_success = false;
        if (enabled && FindTarget() /* && target_player.character_ && target_player.character_->CylinderComponent */) {
            player_world_object.SetLocation(target_player.location_);
            player_world_object.SetVelocity(target_player.velocity_);
            player_world_object.character_ = target_player.character_;

            bool result = abstraction::my_weapon_object.PredictAimAtTarget(&player_world_object, &prediction, FVector());

            if (result) {
                static const bool use_trace = true;
                if (use_trace) {
                    static FVector hit_location;
                    static FVector hit_normal;
                    static FTraceHitInfo hit_info;
                    AActor* hitActor = game_data::local_player_controller->Trace(prediction, target_player.location_, false, FVector(), 0, &hit_location, &hit_normal, &hit_info);

                    if (hitActor) {
                        prediction = hit_location;
                        if (target_player.location_.Z < hit_location.Z) {
                            prediction.Z -= target_player.character_->CylinderComponent->CollisionHeight;
                        } else {
                            prediction.Z += target_player.character_->CylinderComponent->CollisionHeight;
                        }
                    }
                }

                FVector2D projection = game_functions::Project(prediction);
                float height = -1;
                float width = -1;

                if (aimbot_settings.use_triggerbot) {
                    if (projection.X > 0 && projection.Y > 0) {
                        bool los = game_functions::InLineOfSight(target_player.character_);
                        if (los) {
                            FVector2D center_projection = game_functions::Project(target_player.location_);
                            target_player.location_.Z += target_player.character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                            FVector2D head_projection = game_functions::Project(target_player.location_);
                            target_player.location_.Z -= target_player.character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                            height = abs(head_projection.Y - center_projection.Y);
                            width = height * (target_player.character_->CylinderComponent->CollisionRadius / target_player.character_->CylinderComponent->CollisionHeight);

                            if (abs(game_data::screen_center.X - projection.X) < width && abs(game_data::screen_center.Y - projection.Y) < height) {
                                if (game_data::my_player.weapon_ == game_data::Weapon::found) {
                                    // other::SendLeftMouseClick();
                                    game_data::my_player.character_->Weapon->StartFire(fire_mode);
                                    triggerbot_success = true;
                                }
                            }
                        }
                    }
                    if (!triggerbot_success) {
                        game_data::local_player_controller->StopFire(fire_mode);
                    };
                }

                if ((imgui::visuals::aimbot_visual_settings.marker_style == imgui::visuals::MarkerStyle::kBounds || imgui::visuals::aimbot_visual_settings.marker_style == imgui::visuals::MarkerStyle::kFilledBounds) && height == -1) {
                    FVector2D center_projection = game_functions::Project(target_player.location_);
                    target_player.location_.Z += target_player.character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                    FVector2D head_projection = game_functions::Project(target_player.location_);
                    target_player.location_.Z -= target_player.character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                    height = abs(head_projection.Y - center_projection.Y);
                    width = height * (target_player.character_->CylinderComponent->CollisionRadius / target_player.character_->CylinderComponent->CollisionHeight);
                }

                if (projection.Y > 0 && game_functions::IsInFieldOfView(prediction)) {
                    projections_of_predictions.push_back(projection);
                    aimbot_information.push_back({(prediction - game_data::my_player.location_).Magnitude(), projection, height, width});
                }

                if (aimbot_settings.auto_aim || aimbot_enabled) {
                    if (aimbot_settings.aimbot_target_center_of_body)
                        prediction.Z -= target_player.character_->CylinderComponent->CollisionHeight / 1;  // Don't divide by 2?
                    FRotator aim_rotator = math::VectorToRotator(prediction - game_data::my_player.location_);
                    FRotator& aim_rotator_reference = aim_rotator;
                    game_data::local_player_controller->SetRotation(aim_rotator_reference);
                }
            }
        } else {
            if (aimbot_settings.use_triggerbot && !triggerbot_success) {
                game_data::local_player_controller->StopFire(fire_mode);
            }
        }
    } else {
        bool triggerbot_success = false;
        for (vector<game_data::information::Player>::iterator player = game_data::game_data.players.begin(); player != game_data::game_data.players.end(); player++) {
            if (!player->is_valid_ || player->character_ == game_data::my_player.character_) {
                continue;
            }

            game_data::information::Player* p = (game_data::information::Player*)&*player;
            bool same_team = game_data::my_player.IsSameTeam(p);
            // bool same_team = game_functions::IsSameTeam(game_data::local_player_character, player->character_);
            bool line_of_sight = game_functions::InLineOfSight(player->character_);

            if ((same_team && !aimbot_settings.friendly_fire) || (!game_functions::IsInFieldOfView(player->location_) && aimbot_settings.aimbot_mode == AimbotMode::kClosestXhair) || (!line_of_sight && aimbot_settings.need_line_of_sight))
                continue;

            if (!game_functions::IsInHorizontalFieldOfView(player->location_, aimbot_settings.aimbot_horizontal_fov_angle))
                continue;

            player_world_object.SetLocation(player->location_);
            player_world_object.SetVelocity(player->velocity_);
            player_world_object.character_ = player->character_;

            bool result = abstraction::my_weapon_object.PredictAimAtTarget(&player_world_object, &prediction, muzzle_offset);

            if (result) {
                static const bool use_trace = true;
                if (use_trace) {
                    static FVector hit_location;
                    static FVector hit_normal;
                    static FTraceHitInfo hit_info;
                    AActor* hitActor = game_data::local_player_controller->Trace(prediction, player->location_, false, FVector(), 0, &hit_location, &hit_normal, &hit_info);

                    if (hitActor) {
                        prediction = hit_location;
                        if (player->location_.Z < hit_location.Z) {
                            prediction.Z -= player->character_->CylinderComponent->CollisionHeight;
                        } else {
                            prediction.Z += player->character_->CylinderComponent->CollisionHeight;
                        }
                    }
                }

                FVector2D projection = game_functions::Project(prediction);
                float height = -1;
                float width = -1;

                if (aimbot_settings.use_triggerbot) {
                    if (projection.X > 0 && projection.Y > 0) {
                        bool los = game_functions::InLineOfSight(player->character_);
                        if (los) {
                            FVector2D center_projection = game_functions::Project(player->location_);
                            player->location_.Z += player->character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                            FVector2D head_projection = game_functions::Project(player->location_);
                            player->location_.Z -= player->character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                            height = abs(head_projection.Y - center_projection.Y);
                            width = height * (player->character_->CylinderComponent->CollisionRadius / player->character_->CylinderComponent->CollisionHeight);

                            if (abs(game_data::screen_center.X - projection.X) < width && abs(game_data::screen_center.Y - projection.Y) < height) {
                                if (game_data::my_player.weapon_ == game_data::Weapon::found) {
                                    // other::SendLeftMouseClick();
                                    game_data::my_player.character_->Weapon->StartFire(fire_mode);
                                    triggerbot_success = true;
                                }
                            }
                        }
                    }
                }

                if ((imgui::visuals::aimbot_visual_settings.marker_style == imgui::visuals::MarkerStyle::kBounds || imgui::visuals::aimbot_visual_settings.marker_style == imgui::visuals::MarkerStyle::kFilledBounds) && height == -1) {
                    FVector2D center_projection = game_functions::Project(player->location_);
                    player->location_.Z += player->character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                    FVector2D head_projection = game_functions::Project(player->location_);
                    player->location_.Z -= player->character_->CylinderComponent->CollisionHeight;  // this is HALF the height in reality
                    height = abs(head_projection.Y - center_projection.Y);
                    width = height * (player->character_->CylinderComponent->CollisionRadius / player->character_->CylinderComponent->CollisionHeight);
                }

                if (projection.Y >= 0) {
                    projections_of_predictions.push_back(projection);
                    aimbot_information.push_back({(prediction - game_data::my_player.location_).Magnitude(), projection, height, width});
                }

                // math::PrintVector(prediction, "Prediction");
            }
        }

        if (aimbot_settings.use_triggerbot && !triggerbot_success) {
            game_data::local_player_controller->StopFire(fire_mode);
        }
    }

    players_previous = game_data::game_data.players;
}

}  // namespace aimbot

/*
namespace routes {
struct RouteSettings {
    bool enabled = false;
    int route_poll_frequency = 60;
    bool swap_team = false;
    bool find_route_closest_to_spawn = true;
    bool show_route_name = true;
} route_settings;

bool route_file_is_loaded = false;
static json loaded_route_file;

struct RouteTrail {
    string routeName;
    string routeDescription;
    int teamId;
    FVector flagGrabMarker_location;
    vector<FVector> markerLocations;
};

struct Routes {
    string mapName;
    string author;
    vector<struct RouteTrail> routeTrails;

    int selected_route_trail_index = 0;
    const char* route_trail_names[256];
} loaded_routes;

struct SelectedRouteInfo {
} selected_route_info;

bool ParseLoadedRouteFile(void) {
    loaded_routes.mapName = loaded_route_file["mapName"].get<std::string>();
    loaded_routes.author = loaded_route_file["author"].get<std::string>();

    loaded_routes.routeTrails.clear();
    loaded_routes.selected_route_trail_index = 0;

    json route_trails = loaded_route_file["routeTrails"];

    for (json::iterator it = route_trails.begin(); it != route_trails.end(); ++it) {
        RouteTrail route_trail;
        route_trail.routeName = (*it)["routeName"].get<std::string>();
        route_trail.routeDescription = (*it)["routeDescription"].get<std::string>();
        route_trail.teamId = (*it)["teamId"].get<int>();
        route_trail.flagGrabMarker_location.X = (*it)["flagGrabMarker"]["location"]["x"].get<float>();
        route_trail.flagGrabMarker_location.Y = (*it)["flagGrabMarker"]["location"]["y"].get<float>();
        route_trail.flagGrabMarker_location.Z = (*it)["flagGrabMarker"]["location"]["z"].get<float>();

        json marker_locations = (*it)["markerLocations"];
        for (json::iterator it = marker_locations.begin(); it != marker_locations.end(); it++) {
            FVector location;
            location.X = (*it)["location"]["x"].get<float>();
            location.Y = (*it)["location"]["y"].get<float>();
            location.Z = (*it)["location"]["z"].get<float>();
            route_trail.markerLocations.push_back(location);
        }

        loaded_routes.routeTrails.push_back(route_trail);
    }

    for (int i = 0; i < loaded_routes.routeTrails.size(); i++) {
        loaded_routes.route_trail_names[i] = loaded_routes.routeTrails[i].routeName.c_str();
    }

    return false;
}

bool LoadRouteFile(const char* filename) {
    std::ifstream ifs(filename);
    loaded_route_file = json::parse(ifs);
    ParseLoadedRouteFile();
    route_file_is_loaded = true;
    return false;
}

Timer route_draw_poll_timer(route_settings.route_poll_frequency);

vector<FVector2D> projections_of_markers;
// vector<FVector2D> projections_of_markers2;
vector<int> index_of_projections_of_markers;

void Tick(void) {
    if (route_file_is_loaded && route_settings.enabled && game_data::my_player.just_respawned && route_settings.find_route_closest_to_spawn) {
        // loaded_routes.selected_route_trail_index = 0;
        float distance = -1;
        int index = 0;
        for (vector<struct RouteTrail>::iterator i = loaded_routes.routeTrails.begin(); i != loaded_routes.routeTrails.end(); i++) {
            FVector start_location = i->markerLocations[0];
            if (route_settings.swap_team) {
                start_location.X *= -1;
                start_location.Y *= -1;
            }
            float d = (start_location - game_data::my_player.location_).Magnitude();
            if (d < distance || distance < 0) {
                distance = d;
                loaded_routes.selected_route_trail_index = index;
            }
            index++;
        }
    }

    if (!route_draw_poll_timer.isReady() || !route_file_is_loaded || !route_settings.enabled)
        return;

    // route_draw_poll_timer.Tick();

    projections_of_markers.clear();
    // projections_of_markers2.clear();
    index_of_projections_of_markers.clear();

    if (!game_data::my_player.is_valid_)
        ;  // return;

    // Rotator rotation = game_data::my_player_object.GetRotation();
    // Vector rotation_vector = math::RotatorToVector(rotation);

    if (route_settings.enabled && route_file_is_loaded) {
        vector<FVector>* locations = &loaded_routes.routeTrails[loaded_routes.selected_route_trail_index].markerLocations;
        int route_trail_team_id = loaded_routes.routeTrails[loaded_routes.selected_route_trail_index].teamId;

        int j = -1;
        for (vector<FVector>::iterator i = locations->begin(); i != locations->end(); i++) {
            j++;
            FVector marker_location = *i;

            if (route_settings.swap_team) {
                marker_location.X *= -1;
                marker_location.Y *= -1;
            }

            if (!game_functions::IsInFieldOfView(marker_location))
                continue;

            static FVector2D projection_vector;
            projection_vector = game_functions::Project(marker_location);
            projections_of_markers.push_back(projection_vector);
            index_of_projections_of_markers.push_back(j);
        }
    }
}

void NextRouteTrail(void) {
    if (!route_file_is_loaded) {
        return;
    }
    if (loaded_routes.routeTrails.size() > 0) {
        loaded_routes.selected_route_trail_index = (loaded_routes.selected_route_trail_index + 1) % loaded_routes.routeTrails.size();
    }
}

void PreviousRouteTrail(void) {
    if (!route_file_is_loaded) {
        return;
    }
    if (loaded_routes.routeTrails.size() > 0) {
        loaded_routes.selected_route_trail_index--;
        if (loaded_routes.selected_route_trail_index < 0) {
            loaded_routes.selected_route_trail_index += loaded_routes.routeTrails.size();
        }
    }
}

}  // namespace routes
*/

namespace radar {
static struct RadarSettings {
    bool enabled = true;
    int radar_poll_frequency = 60;
    bool show_friendlies = false;
    bool show_flags = true;
    // ImVec2 window_size = {300, 300};
} radar_settings;

static Timer get_radar_data_timer(radar_settings.radar_poll_frequency);

struct RadarLocation {  // polar coordinates
    float r = 0;
    float theta = 0;
    bool right = 0;

    /*
    void Clear(void) {
                    r = 0;
                    theta = 0;
                    right = 0;
    }
    */
};

struct RadarInformation : RadarLocation {
    bool is_friendly = false;
};

static vector<RadarInformation> player_locations;
static vector<RadarInformation> flag_locations;

static RadarLocation friendly_flag_location;
static RadarLocation enemy_flag_location;

void Tick(void) {
    if (!radar_settings.enabled || !get_radar_data_timer.isReady())
        return;

    player_locations.clear();
    flag_locations.clear();
    // friendly_flag_location.Clear();
    // enemy_flag_location.Clear();

    if (!game_data::my_player.is_valid_)
        ;  // return;

    for (vector<game_data::information::Player>::iterator player = game_data::game_data.players.begin(); player != game_data::game_data.players.end(); player++) {
        if (!player->is_valid_ || player->character_ == game_data::my_player.character_ || !validate::IsValid(player->character_)) {
            continue;
        }

        game_data::information::Player* p = (game_data::information::Player*)&*player;
        bool same_team = game_data::my_player.IsSameTeam(p);
        // bool same_team = game_functions::IsSameTeam(game_data::my_player.character_, player->character_);
        if (same_team && !radar_settings.show_friendlies) {
            continue;
        }

        FVector difference_vector = player->location_ - game_data::my_player.location_;
        float angle = math::AngleBetweenVector(game_data::my_player.forward_vector_, difference_vector);
        float magnitude = difference_vector.Magnitude();
        bool right = math::IsVectorToRight(game_data::my_player.forward_vector_, difference_vector);
        RadarInformation radar_information = {magnitude, angle, right, same_team};

        float delta = magnitude * imgui::visuals::radar_visual_settings.zoom_ * imgui::visuals::radar_visual_settings.zoom;
        if (delta > imgui::visuals::radar_visual_settings.window_size / 2 - imgui::visuals::radar_visual_settings.border) {
            continue;
        }

        player_locations.push_back(radar_information);
    }

    if (radar_settings.show_flags) {
    }
}

}  // namespace radar

namespace game_data {
void GetWeapon(void) {
    if (!my_player.is_valid_) {
        return;
    }

    ATrDevice* weapon = (ATrDevice*)my_player.character_->Weapon;
    if (weapon) {
        game_data::my_player.weapon_ = game_data::Weapon::found;

        bool projectile_weapon = weapon->IsA(ATrDevice_ThrowingKnives::StaticClass()) || weapon->IsA(ATrDevice_SN7::StaticClass()) || weapon->IsA(ATrDevice_SN7_MKD::StaticClass());

        if (!projectile_weapon && weapon->bInstantHit && !weapon->IsA(ATrDevice_ConstantFire::StaticClass())) {
            abstraction::my_weapon_object.SetWeaponType(abstraction::WeaponObject::WeaponType::kHitscan);
        } else {
            static abstraction::WeaponObject::WeaponParameters* weapon_parameters = abstraction::my_weapon_object.GetWeaponParameters();
            static abstraction::WeaponObject::WeaponAimbotParameters* aimbot_parameters = abstraction::my_weapon_object.GetAimbotParameters();
            if (aimbot::aimbot_settings.use_custom_ping) {
                weapon_parameters->ping_ = aimbot::aimbot_settings.ping_in_ms;
            } else {
                weapon_parameters->ping_ = game_data::my_player.character_->PlayerReplicationInfo->ExactPing * 1000;
                // cout << "Exact Ping" << game_data::my_player.character_->PlayerReplicationInfo->ExactPing << endl;
            }

            ATrProjectile* p = (ATrProjectile*)weapon->Spawn(weapon->WeaponProjectiles.Data[0], weapon, FName(0), FVector({-999999, -999999, -999999}), FRotator(), nullptr, 0);
            if (p) {
                weapon_parameters->bullet_speed_ = p->Speed;
                weapon_parameters->inheritence_ = p->m_fMaxProjInheritPct;
                abstraction::my_weapon_object.SetWeaponType(abstraction::WeaponObject::WeaponType::kProjectileLinear);

                p->ImpactSound = NULL;
                p->SpawnSound = NULL;
                p->SetCollisionSize(0, 0);
                p->bHidden = true;
                p->Velocity = {0, 0, 0};
                p->Destroy();

            } else {
                game_data::my_player.weapon_ = game_data::Weapon::unknown;
            }
        }
    } else {
        my_player.weapon_ == Weapon::none;
        abstraction::my_weapon_object.SetWeaponType(abstraction::WeaponObject::WeaponType::kHitscan);
    }
}
}  // namespace game_data

namespace config {

static bool freshly_loaded_config = false;

void WriteDataToFile(void* data, int size, FILE* file) {
    fwrite(&size, sizeof(int), 1, file);
    fwrite(data, size, 1, file);
}

void ReadDataFromFile(void* data, int size, FILE* file) {
    int struct_size;
    fread(&struct_size, sizeof(int), 1, file);
    fread(data, struct_size, 1, file);
}

bool SaveConfig(const char* filename) {
    FILE* save_config_file = fopen(filename, "wb");
    FILE* file = save_config_file;

    if (save_config_file) {
        using namespace imgui;

        int struct_size = 0;

        // Imgui menu settings
        WriteDataToFile(&imgui::imgui_settings, sizeof(imgui::imgui_settings), file);

        // Aimbot settings
        WriteDataToFile(&aimbot::aimbot_settings, sizeof(aimbot::aimbot_settings), file);
        WriteDataToFile(&visuals::aimbot_visual_settings, sizeof(visuals::aimbot_visual_settings), file);

        // Route settings
        // WriteDataToFile(&routes::route_settings, sizeof(routes::route_settings), file);
        WriteDataToFile(&visuals::route_visual_settings, sizeof(visuals::route_visual_settings), file);

        // ESP settings
        WriteDataToFile(&esp::esp_settings, sizeof(esp::esp_settings), file);
        WriteDataToFile(&visuals::esp_visual_settings, sizeof(visuals::esp_visual_settings), file);

        // Radar settings
        WriteDataToFile(&radar::radar_settings, sizeof(radar::radar_settings), file);
        WriteDataToFile(&visuals::radar_visual_settings, sizeof(visuals::radar_visual_settings), file);

        // Other settings
        WriteDataToFile(&other::other_settings, sizeof(other::other_settings), file);

        // Crosshair settings
        WriteDataToFile(&visuals::crosshair_settings, sizeof(visuals::crosshair_settings), file);

        fclose(save_config_file);
        return true;
    }
    return false;
}

bool LoadConfig(const char* filename) {
    LOG2("Loading config", string(filename));
    FILE* load_config_file = fopen(filename, "rb");
    FILE* file = load_config_file;

    if (load_config_file) {
        using namespace imgui;

        int struct_size = 0;

        // Imgui menu settings
        ReadDataFromFile(&imgui::imgui_settings, sizeof(imgui::imgui_settings), file);

        // Aimbot settings
        ReadDataFromFile(&aimbot::aimbot_settings, sizeof(aimbot::aimbot_settings), file);
        ReadDataFromFile(&visuals::aimbot_visual_settings, sizeof(visuals::aimbot_visual_settings), file);

        // Route settings
        // ReadDataFromFile(&routes::route_settings, sizeof(routes::route_settings), file);
        ReadDataFromFile(&visuals::route_visual_settings, sizeof(visuals::route_visual_settings), file);

        // ESP settings
        ReadDataFromFile(&esp::esp_settings, sizeof(esp::esp_settings), file);
        ReadDataFromFile(&visuals::esp_visual_settings, sizeof(visuals::esp_visual_settings), file);

        // Radar settings
        ReadDataFromFile(&radar::radar_settings, sizeof(radar::radar_settings), file);
        ReadDataFromFile(&visuals::radar_visual_settings, sizeof(visuals::radar_visual_settings), file);

        // Other settings
        ReadDataFromFile(&other::other_settings, sizeof(other::other_settings), file);

        // Crosshair settings
        ReadDataFromFile(&visuals::crosshair_settings, sizeof(visuals::crosshair_settings), file);

        aimbot::aimbot_poll_timer.SetFrequency(aimbot::aimbot_settings.aimbot_poll_frequency);
        // routes::route_draw_poll_timer.SetFrequency(routes::route_settings.route_poll_frequency);
        radar::get_radar_data_timer.SetFrequency(radar::radar_settings.radar_poll_frequency);
        esp::get_esp_data_timer.SetFrequency(esp::esp_settings.poll_frequency);
        esp::esp_settings.show_names = false;

        fclose(load_config_file);

        freshly_loaded_config = true;
        return true;
    }
    return false;
}
}  // namespace config

namespace imgui {
namespace imgui_menu {
enum LeftMenuButtons { kAimAssist, kAimbot, kESP, kRadar, kOther, kAimTracker, kRoutes, kOptions, kCrosshair, kSkinChanger, kConfigs, kCredits, kLua, kBindings, kPID, kTraining, kInformation };
static const char* button_text[] = {"Aim assist", "-", "ESP", "Radar", "Other", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "Information"};
// static const char* button_text[] = {"-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "Scripting", "-", "-"};
static const int buttons_num = sizeof(button_text) / sizeof(char*);
static int selected_index = LeftMenuButtons::kInformation;  // LeftMenuButtons::kAimAssist;
static float item_width = -150;

void DrawAimAssistMenu(void) {
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | (ImGuiTableFlags_ContextMenuInBody & 0) | (ImGuiTableFlags_NoBordersInBody & 0) | ImGuiTableFlags_BordersOuter;
    if (ImGui::BeginTable("aimassisttable", 1, flags, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("Aim assist", ImGuiTableColumnFlags_WidthStretch);
        // ImGui::TableSetupColumn("Visuals", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushItemWidth(item_width);
        if (ImGui::CollapsingHeader("General settings")) {
            ImGui::Indent();

            // ImGui::SliderInt("FireMode", &aimbot::fire_mode, 0, 255);

            ImGui::Checkbox("Enable aim assist", &aimbot::aimbot_settings.enabled);
            // ImGui::PushItemWidth(item_width * 2);
            ImGui::Combo("Mode##aim_assist_mode_combo", (int*)&aimbot::aimbot_settings.aimbot_mode, aimbot::mode_labels, IM_ARRAYSIZE(aimbot::mode_labels));
            // ImGui::PopItemWidth();

            if (aimbot::aimbot_settings.aimbot_mode == aimbot::AimbotMode::kClosestXhair) {
                ImGui::SliderFloat("Horizontal FOV", &aimbot::aimbot_settings.aimbot_horizontal_fov_angle, 1, 89);
                aimbot::aimbot_settings.aimbot_horizontal_fov_angle_cos = cos(aimbot::aimbot_settings.aimbot_horizontal_fov_angle * PI / 180.0);
                aimbot::aimbot_settings.aimbot_horizontal_fov_angle_cos_sqr = pow(aimbot::aimbot_settings.aimbot_horizontal_fov_angle_cos, 2);
            }

            if (ImGui::SliderInt("Poll rate (Hz)", &aimbot::aimbot_settings.aimbot_poll_frequency, 1, 300)) {
                aimbot::aimbot_poll_timer.SetFrequency(aimbot::aimbot_settings.aimbot_poll_frequency);
            }

            ImGui::Checkbox("Factor target acceleration", &aimbot::aimbot_settings.use_acceleration);
#ifdef CAP_ACCELERATION
            if (aimbot::aimbot_settings.use_acceleration) {
                ImGui::Indent();
                ImGui::SliderFloat("Acceleration cap (X)", &aimbot::aimbot_settings.acceleration_cap_x, 1, 1E4);
                ImGui::SliderFloat("Acceleration cap (Y)", &aimbot::aimbot_settings.acceleration_cap_y, 1, 1E4);
                ImGui::SliderFloat("Acceleration cap (Z)", &aimbot::aimbot_settings.acceleration_cap_z, 1, 1E4);
                ImGui::Unindent();
            }
#endif

            // ImGui::Checkbox("Use trigger bot (Explosives only)", &aimbot::aimbot_settings.use_triggerbot);

            ImGui::Checkbox("Use custom ping value", &aimbot::aimbot_settings.use_custom_ping);

            if (aimbot::aimbot_settings.use_custom_ping) {
                ImGui::SliderFloat("Custom projectile ping", &aimbot::aimbot_settings.ping_in_ms, -400, 400);
                ImGui::PopItemWidth();
            }

            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Target settings")) {
            ImGui::Indent();
            ImGui::Checkbox("Friendly fire", &aimbot::aimbot_settings.friendly_fire);
            ImGui::Checkbox("Need line of sight", &aimbot::aimbot_settings.need_line_of_sight);
            ImGui::Checkbox("Target everyone", &aimbot::aimbot_settings.target_everyone);
            if (!aimbot::aimbot_settings.target_everyone) {
                ImGui::Checkbox("Stay locked on to target", &aimbot::aimbot_settings.stay_locked_to_target);
                ImGui::Checkbox("Auto lock to new target", &aimbot::aimbot_settings.auto_lock_to_new_target);
                ImGui::Checkbox("Aimbot targets center of body", &aimbot::aimbot_settings.aimbot_target_center_of_body);
                // ImGui::Checkbox("Auto aim", &aimbot::aimbot_settings.auto_aim);
            }
            ImGui::Unindent();
        }

        /*
        if (ImGui::CollapsingHeader("Weapon settings")) {
            ImGui::Indent();
            if (ImGui::CollapsingHeader("Pings")) {
                ImGui::Indent();
                ImGui::SliderFloat("Tempest ping", &aimbot::aimbot_settings.tempest_ping_in_ms, -300, 300);
                ImGui::SliderFloat("Chaingun ping", &aimbot::aimbot_settings.chaingun_ping_in_ms, -300, 300);
                ImGui::SliderFloat("Grenade Launcher ping", &aimbot::aimbot_settings.grenadelauncher_ping_in_ms, -300, 300);
                ImGui::SliderFloat("Plasma Gun ping", &aimbot::aimbot_settings.plasmagun_ping_in_ms, -300, 300);
                ImGui::SliderFloat("Blaster ping", &aimbot::aimbot_settings.blaster_ping_in_ms, -300, 300);
                // ImGui::SliderFloat("Self compensation ping", &aimbot::aimbot_settings.self_compensation_time_in_ms, -300, 300);
                ImGui::Unindent();
            }

            if (ImGui::CollapsingHeader("Bullet speeds")) {
                ImGui::Indent();
                ImGui::SliderFloat("Tempest speed", &game_data::information::weapon_speeds.disk.bullet_speed, 0, 1E5);
                ImGui::SliderFloat("Chaingun speed", &game_data::information::weapon_speeds.chaingun.bullet_speed, 0, 1E5);
                ImGui::SliderFloat("Grenadelauncher speed", &game_data::information::weapon_speeds.grenadelauncher.bullet_speed, 0, 1E5);
                ImGui::SliderFloat("Plasma speed", &game_data::information::weapon_speeds.plasma.bullet_speed, 0, 1E5);
                ImGui::SliderFloat("Blaster speed", &game_data::information::weapon_speeds.blaster.bullet_speed, 0, 1E5);
                ImGui::Unindent();
            }

            if (ImGui::CollapsingHeader("Inheritence")) {
                ImGui::Indent();
                ImGui::SliderFloat("Tempest inheritence", &game_data::information::weapon_speeds.disk.inheritence, 0, 1);
                ImGui::SliderFloat("Chaingun inheritence", &game_data::information::weapon_speeds.chaingun.inheritence, 0, 1);
                ImGui::SliderFloat("Grenadelauncher inheritence", &game_data::information::weapon_speeds.grenadelauncher.inheritence, 0, 1);
                ImGui::SliderFloat("Plasma inheritence", &game_data::information::weapon_speeds.plasma.inheritence, 0, 1);
                ImGui::SliderFloat("Blaster inheritence", &game_data::information::weapon_speeds.blaster.inheritence, 0, 1);
                ImGui::Unindent();
            }

            ImGui::Unindent();
        }
        */

        if (ImGui::CollapsingHeader("Markers")) {
            // ImGui::TableSetColumnIndex(1);
            // ImGui::PushItemWidth(item_width);
            float marker_preview_size = 100;
            ImGui::Combo("Style##aim_assist_combo", (int*)&visuals::aimbot_visual_settings.marker_style, visuals::marker_labels, IM_ARRAYSIZE(visuals::marker_labels));
            ImGui::ColorEdit4("Colour", &visuals::aimbot_visual_settings.marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);

            if (visuals::aimbot_visual_settings.marker_style != visuals::MarkerStyle::kBounds && visuals::aimbot_visual_settings.marker_style != visuals::MarkerStyle::kFilledBounds)
                ImGui::SliderInt("Radius", &visuals::aimbot_visual_settings.marker_size, 1, 10);

            if (visuals::aimbot_visual_settings.marker_style == visuals::MarkerStyle::kCircle || visuals::aimbot_visual_settings.marker_style == visuals::MarkerStyle::kSquare || visuals::aimbot_visual_settings.marker_style == visuals::MarkerStyle::kBounds) {
                ImGui::SliderInt("Thickness", &visuals::aimbot_visual_settings.marker_thickness, 1, 10);
            }

            ImGui::Text("Marker preview");

            ImVec2 window_position = ImGui::GetWindowPos();
            ImVec2 window_size = ImGui::GetWindowSize();

            ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();
            ImVec2 current_cursor_pos = ImGui::GetCursorPos();
            ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY() - ImGui::GetScrollY()};
            imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + marker_preview_size, local_cursor_pos.y + marker_preview_size}, ImColor(0, 0, 0, 255), 0, 0);
            ImVec2 center = {local_cursor_pos.x + marker_preview_size / 2, local_cursor_pos.y + marker_preview_size / 2};

            int marker_style = visuals::aimbot_visual_settings.marker_style;
            int marker_size = visuals::aimbot_visual_settings.marker_size;
            int marker_thickness = visuals::aimbot_visual_settings.marker_thickness;
            ImColor marker_colour = visuals::aimbot_visual_settings.marker_colour;

            float box_size_height = 40;
            float box_size_width = box_size_height * esp::esp_settings.width_to_height_ratio;

            switch (visuals::aimbot_visual_settings.marker_style) {
                case visuals::MarkerStyle::kDot:
                    imgui_draw_list->AddCircleFilled(center, marker_size, marker_colour);
                    break;
                case visuals::MarkerStyle::kCircle:
                    imgui_draw_list->AddCircle(center, marker_size, marker_colour, 0, marker_thickness);
                    break;
                case visuals::MarkerStyle::kFilledSquare:
                    imgui_draw_list->AddRectFilled({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0);
                    break;
                case visuals::MarkerStyle::kSquare:
                    imgui_draw_list->AddRect({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0, 0, marker_thickness);
                    break;
                case visuals::MarkerStyle::kBounds:
                    imgui_draw_list->AddRect({center.x - box_size_width, center.y - box_size_height}, {center.x + box_size_width, center.y + box_size_height}, marker_colour, 0, 0, marker_thickness);
                    break;
                case visuals::MarkerStyle::kFilledBounds:
                    imgui_draw_list->AddRectFilled({center.x - box_size_width, center.y - box_size_height}, {center.x + box_size_width, center.y + box_size_height}, marker_colour, 0);
                    break;
                default:
                    break;
            }

            current_cursor_pos.y += marker_preview_size;
            ImGui::SetCursorPos(current_cursor_pos);
            if (visuals::aimbot_visual_settings.marker_style != visuals::MarkerStyle::kBounds && visuals::aimbot_visual_settings.marker_style != visuals::MarkerStyle::kFilledBounds) {
                ImGui::Spacing();
                ImGui::Checkbox("Scale by distance", &visuals::aimbot_visual_settings.scale_by_distance);
                if (visuals::aimbot_visual_settings.scale_by_distance) {
                    ImGui::SliderInt("Distance for scaling", &visuals::aimbot_visual_settings.distance_for_scaling, 1, 15000);
                    ImGui::SliderInt("Minimum marker size", &visuals::aimbot_visual_settings.minimum_marker_size, 1, 10);
                }
            }
        }

        if (ImGui::CollapsingHeader("Triggerbot settings")) {
            ImGui::Indent();
            if (ImGui::Checkbox("Enable triggerbot", &aimbot::aimbot_settings.use_triggerbot)) {
                if (game_data::local_player_controller)
                    game_data::local_player_controller->StopFire(aimbot::fire_mode);
            }
            ImGui::Unindent();
        }

        ImGui::EndTable();
    }
}

/*
void DrawRoutesMenu(void) {
    return;
    static Timer routes_file_check_timer(5);
    int right_child_width = 200;

    ImGui::BeginGroup();
    ImGui::BeginChild("left_settings##left_settings", ImVec2(ImGui::GetWindowContentRegionWidth() > right_child_width ? ImGui::GetWindowContentRegionWidth() - right_child_width : ImGui::GetWindowContentRegionWidth(), 0));

    ImGui::Text("Settings");
    ImGui::Separator();
    ImGui::Checkbox("Enabled##route_enabled", &routes::route_settings.enabled);
    ImGui::PushItemWidth(100);
    if (ImGui::SliderInt("Poll rate (Hz)##routes", &routes::route_settings.route_poll_frequency, 1, 300)) {
        routes::route_draw_poll_timer.SetFrequency(routes::route_settings.route_poll_frequency);
    }
    ImGui::Checkbox("Swap team (Mirror)", &routes::route_settings.swap_team);
    ImGui::PopItemWidth();

    ImGui::Checkbox("Auto pick route closest to spawn", &routes::route_settings.find_route_closest_to_spawn);
    ImGui::Checkbox("Show route name", &routes::route_settings.show_route_name);

    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(2 * 0.05, 2 * 0.05, 2 * 0.05, 2 * 0.1));
    ImGui::Separator();
    ImGui::PopStyleColor();

    ImGui::Text("Routes");
    ImGui::Separator();

    static vector<UsefulSnippets::Files::FileObject> route_files = UsefulSnippets::Files::getFiles("routes", ".txt", true);

    static const char* route_filenames[256];
    static const char* route_paths[256];

    if (routes_file_check_timer.isReady()) {
        int c = 0;
        for (vector<UsefulSnippets::Files::FileObject>::iterator i = route_files.begin(); i != route_files.end(); i++) {
            // cout << "Route found: " << i->getFileName_cstr() << endl;
            route_filenames[c] = i->getFileName_cstr();
            route_paths[c] = i->getFilePath_cstr();
            c++;
        }

        // routes_file_check_timer.Tick();
    }
    static int route_file_index = 0;
    if (route_file_index > route_files.size() - 1) {
        route_file_index = 0;
    }

    ImGui::PushItemWidth(100);
    if (ImGui::Combo("Route files##routes_load", &route_file_index, route_filenames, route_files.size())) {
    }
    ImGui::PopItemWidth();

    if (route_files.size() > 0) {
        if (ImGui::Button("Load route file##route_load")) {
            routes::LoadRouteFile(route_paths[route_file_index]);
        }
    } else {
        ImGui::Text("No route files were found.");
    }

    if (routes::route_file_is_loaded) {
        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(2 * 0.05, 2 * 0.05, 2 * 0.05, 2 * 0.1));
        ImGui::Separator();
        ImGui::PopStyleColor();

        ImGui::Text("Draw route trail");
        ImGui::Separator();

        ImGui::PushItemWidth(100);
        if (ImGui::Combo("Route trails##trail_combo", &routes::loaded_routes.selected_route_trail_index, routes::loaded_routes.route_trail_names, routes::loaded_routes.routeTrails.size())) {
            //
        }
        ImGui::PopItemWidth();
    } else {
        ImGui::Text("No route file currently loaded.");
    }

    ImGui::EndChild();
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::SetCursorPosX({ImGui::GetCursorPos().x + 10});
    ImGui::BeginGroup();
    ImGui::BeginChild("right_settings##right_settings", ImVec2(right_child_width, 0));

    ImGui::Text("Marker settings");
    ImGui::Separator();
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);

    ImGui::Combo("Style##aim_assist_combo", (int*)&visuals::route_visual_settings.marker_style, visuals::marker_labels, IM_ARRAYSIZE(visuals::marker_labels));
    ImGui::ColorEdit4("Colour", &visuals::route_visual_settings.marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::Checkbox("Rainbow gradient", &visuals::route_visual_settings.rainbow_gradient);
    ImGui::SliderInt("Radius", &visuals::route_visual_settings.marker_size, 1, 50);

    if (visuals::route_visual_settings.marker_style == visuals::MarkerStyle::kCircle || visuals::route_visual_settings.marker_style == visuals::MarkerStyle::kSquare) {
        ImGui::SliderInt("Thickness", &visuals::route_visual_settings.marker_thickness, 1, 10);
    }

    ImGui::Text("Marker preview");

    ImVec2 window_position = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();

    ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();
    ImVec2 current_cursor_pos = ImGui::GetCursorPos();
    ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY()};
    imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + 100, local_cursor_pos.y + 100}, ImColor(0, 0, 0, 255), 0, 0);
    ImVec2 center = {local_cursor_pos.x + 100 / 2, local_cursor_pos.y + 100 / 2};

    int marker_style = visuals::route_visual_settings.marker_style;
    int marker_size = visuals::route_visual_settings.marker_size;
    int marker_thickness = visuals::route_visual_settings.marker_thickness;
    ImColor marker_colour = visuals::route_visual_settings.marker_colour;

    switch (visuals::route_visual_settings.marker_style) {
        case visuals::MarkerStyle::kDot:
            imgui_draw_list->AddCircleFilled(center, marker_size, marker_colour);
            break;
        case visuals::MarkerStyle::kCircle:
            imgui_draw_list->AddCircle(center, marker_size, marker_colour, 0, marker_thickness);
            break;
        case visuals::MarkerStyle::kFilledSquare:
            imgui_draw_list->AddRectFilled({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0);
            break;
        case visuals::MarkerStyle::kSquare:
            imgui_draw_list->AddRect({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0, 0, marker_thickness);
            break;
        default:
            break;
    }

    ImGui::EndChild();
    ImGui::EndGroup();
}
*/

void DrawRadarMenu(void) {
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | (ImGuiTableFlags_ContextMenuInBody & 0) | (ImGuiTableFlags_NoBordersInBody & 0) | ImGuiTableFlags_BordersOuter;
    if (ImGui::BeginTable("radartable", 1, flags, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("Radar", ImGuiTableColumnFlags_WidthStretch);
        // ImGui::TableSetupColumn("Visuals", ImGuiTableColumnFlags_WidthFixed);
        // ImGui::TableSetupColumn("Visuals", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushItemWidth(item_width);
        if (ImGui::CollapsingHeader("General settings")) {
            ImGui::Indent();
            ImGui::Checkbox("Enabled##radar_enabled", &radar::radar_settings.enabled);
            if (ImGui::SliderInt("Poll rate (Hz)##routes", &radar::radar_settings.radar_poll_frequency, 1, 300)) {
                radar::get_radar_data_timer.SetFrequency(radar::radar_settings.radar_poll_frequency);
            }

            ImGui::SliderFloat("Zoom", &visuals::radar_visual_settings.zoom, 0, 10);
            ImGui::Checkbox("Show friendlies", &radar::radar_settings.show_friendlies);
            ImGui::Checkbox("Show flags", &radar::radar_settings.show_flags);
            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Visual settings")) {
            ImGui::Indent();
            ImGui::ColorEdit4("Backgrond Colour", &visuals::radar_visual_settings.window_background_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::Checkbox("Draw axes", &visuals::radar_visual_settings.draw_axes);
            if (visuals::radar_visual_settings.draw_axes)
                ImGui::SliderInt("Axes thickness", &visuals::radar_visual_settings.axes_thickness, 1, 4);
            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Markers")) {
            ImGui::Indent();
            float marker_preview_size = 100;

            ImGui::Combo("Style##radar_combo", (int*)&visuals::radar_visual_settings.marker_style, visuals::marker_labels, IM_ARRAYSIZE(visuals::marker_labels));
            ImGui::SliderInt("Radius", &visuals::radar_visual_settings.marker_size, 1, 50);

            if (visuals::radar_visual_settings.marker_style == visuals::MarkerStyle::kCircle || visuals::radar_visual_settings.marker_style == visuals::MarkerStyle::kSquare) {
                ImGui::SliderInt("Thickness", &visuals::radar_visual_settings.marker_thickness, 1, 10);
            }

            ImGui::Text("Marker preview");

            ImVec2 window_position = ImGui::GetWindowPos();
            ImVec2 window_size = ImGui::GetWindowSize();

            ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();
            ImVec2 current_cursor_pos = ImGui::GetCursorPos();
            ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY() - ImGui::GetScrollY()};
            imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + marker_preview_size, local_cursor_pos.y + marker_preview_size}, ImColor(0, 0, 0, 255), 0, 0);
            ImVec2 center = {local_cursor_pos.x + marker_preview_size / 2, local_cursor_pos.y + marker_preview_size / 2};

            int marker_style = visuals::radar_visual_settings.marker_style;
            int marker_size = visuals::radar_visual_settings.marker_size;
            int marker_thickness = visuals::radar_visual_settings.marker_thickness;
            ImColor marker_colour = visuals::radar_visual_settings.enemy_marker_colour;

            switch (visuals::radar_visual_settings.marker_style) {
                case visuals::MarkerStyle::kDot:
                    imgui_draw_list->AddCircleFilled(center, marker_size, marker_colour);
                    break;
                case visuals::MarkerStyle::kCircle:
                    imgui_draw_list->AddCircle(center, marker_size, marker_colour, 0, marker_thickness);
                    break;
                case visuals::MarkerStyle::kFilledSquare:
                    imgui_draw_list->AddRectFilled({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0);
                    break;
                case visuals::MarkerStyle::kSquare:
                    imgui_draw_list->AddRect({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0, 0, marker_thickness);
                    break;
                default:
                    break;
            }

            current_cursor_pos.y += marker_preview_size;
            ImGui::SetCursorPos(current_cursor_pos);
            ImGui::Spacing();

            // ImGui::Separator();
            ImGui::ColorEdit4("Friendly player Colour", &visuals::radar_visual_settings.friendly_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::ColorEdit4("Enemy player Colour", &visuals::radar_visual_settings.enemy_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::ColorEdit4("Friendly flag Colour", &visuals::radar_visual_settings.friendly_flag_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::ColorEdit4("Enemy flag Colour", &visuals::radar_visual_settings.enemy_flag_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::Unindent();
        }

        // ImGui::TableSetColumnIndex(1);
        // ImGui::PushItemWidth(item_width);

        ImGui::EndTable();
    }
}

void DrawESPMenu(void) {
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | (ImGuiTableFlags_ContextMenuInBody & 0) | (ImGuiTableFlags_NoBordersInBody & 0) | ImGuiTableFlags_BordersOuter;
    if (ImGui::BeginTable("esptable", 1, flags, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("ESP", ImGuiTableColumnFlags_WidthStretch);
        // ImGui::TableSetupColumn("Visuals", ImGuiTableColumnFlags_WidthFixed);
        // ImGui::TableSetupColumn("Visuals", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushItemWidth(item_width);
        if (ImGui::CollapsingHeader("General settings")) {
            ImGui::Indent();
            ImGui::Checkbox("Enabled", &esp::esp_settings.enabled);
            if (ImGui::SliderInt("Poll rate (Hz)", &esp::esp_settings.poll_frequency, 1, 300)) {
                esp::get_esp_data_timer.SetFrequency(esp::esp_settings.poll_frequency);
            }
            ImGui::Checkbox("Show friendlies", &esp::esp_settings.show_friendlies);

            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("BoundingBox settings")) {
            ImGui::Indent();
            ImGui::SliderInt("Box thickness", &visuals::esp_visual_settings.bounding_box_settings.box_thickness, 1, 20);
            ImGui::ColorEdit4("Friendly Colour##box", &visuals::esp_visual_settings.bounding_box_settings.friendly_player_box_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::ColorEdit4("Enemy Colour##box", &visuals::esp_visual_settings.bounding_box_settings.enemy_player_box_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Snapline settings")) {
            ImGui::Indent();
            ImGui::Checkbox("Show lines", &esp::esp_settings.show_lines);
            ImGui::ColorEdit4("Enemy Colour##line", &visuals::esp_visual_settings.line_settings.enemy_player_line_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
            ImGui::SliderInt("Line thickness", &visuals::esp_visual_settings.line_settings.line_thickness, 1, 20);
            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Player name settings")) {
            ImGui::Indent();
            ImGui::Checkbox("Show player names", &esp::esp_settings.show_names);
            if (esp::esp_settings.show_names) {
                ImGui::Indent();
                ImGui::ColorEdit4("Friendly name colour##name", &visuals::esp_visual_settings.name_settings.friendly_name_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
                ImGui::ColorEdit4("Enemy name colour##name", &visuals::esp_visual_settings.name_settings.enemy_name_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
                ImGui::Unindent();
            }
            ImGui::Unindent();
        }

        // ImGui::TableSetColumnIndex(1);
        // ImGui::PushItemWidth(item_width);

        ImGui::EndTable();
    }
}

void DrawCrosshairMenu(void) {
    int right_child_width = 200;

    ImGui::BeginGroup();
    ImGui::BeginChild("left_settings##left_settings", ImVec2(ImGui::GetWindowContentRegionWidth() > right_child_width ? ImGui::GetWindowContentRegionWidth() - right_child_width : ImGui::GetWindowContentRegionWidth(), 0));

    ImGui::Text("Settings");
    ImGui::Separator();
    ImGui::Checkbox("Enabled##crosshair_enabled", &visuals::crosshair_settings.show_or_enabled);

    ImGui::EndChild();
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::SetCursorPosX({ImGui::GetCursorPos().x + 10});
    ImGui::BeginGroup();
    ImGui::BeginChild("right_settings##right_settings", ImVec2(right_child_width, 0));

    ImGui::Text("Marker settings");
    ImGui::Separator();
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);

    ImGui::Combo("Style##crosshair_combo", (int*)&visuals::crosshair_settings.marker_style, visuals::marker_labels, IM_ARRAYSIZE(visuals::marker_labels));
    ImGui::ColorEdit4("Colour", &visuals::crosshair_settings.marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::SliderInt("Radius", &visuals::crosshair_settings.marker_size, 1, 50);

    if (visuals::crosshair_settings.marker_style == visuals::MarkerStyle::kCircle || visuals::crosshair_settings.marker_style == visuals::MarkerStyle::kSquare) {
        ImGui::SliderInt("Thickness", &visuals::crosshair_settings.marker_thickness, 1, 10);
    }

    ImGui::Text("Marker preview");

    ImVec2 window_position = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();

    ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();
    ImVec2 current_cursor_pos = ImGui::GetCursorPos();
    ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY()};
    imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + 100, local_cursor_pos.y + 100}, ImColor(0, 0, 0, 255), 0, 0);
    ImVec2 center = {local_cursor_pos.x + 100 / 2, local_cursor_pos.y + 100 / 2};

    int marker_style = visuals::crosshair_settings.marker_style;
    int marker_size = visuals::crosshair_settings.marker_size;
    int marker_thickness = visuals::crosshair_settings.marker_thickness;
    ImColor marker_colour = visuals::crosshair_settings.marker_colour;

    switch (visuals::crosshair_settings.marker_style) {
        case visuals::MarkerStyle::kDot:
            imgui_draw_list->AddCircleFilled(center, marker_size, marker_colour);
            break;
        case visuals::MarkerStyle::kCircle:
            imgui_draw_list->AddCircle(center, marker_size, marker_colour, 0, marker_thickness);
            break;
        case visuals::MarkerStyle::kFilledSquare:
            imgui_draw_list->AddRectFilled({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0);
            break;
        case visuals::MarkerStyle::kSquare:
            imgui_draw_list->AddRect({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0, 0, marker_thickness);
            break;
        default:
            break;
    }

    ImGui::EndChild();
    ImGui::EndGroup();
}

/*
void DrawConfigMenu(void) {
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | (ImGuiTableFlags_ContextMenuInBody & 0) | (ImGuiTableFlags_NoBordersInBody & 0) | ImGuiTableFlags_BordersOuter;
    if (ImGui::BeginTable("configtable", 1, flags, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("Configs", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushItemWidth(item_width);

        if (ImGui::CollapsingHeader("Save config")) {
            ImGui::Indent();
            static char filename_buffer[256] = "default.cfg";
            ImGui::InputText("File name##save_config_file", filename_buffer, 255);
            if (ImGui::Button("Save config##save_config_file")) {
                config::SaveConfig(filename_buffer);
            }
            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Load config")) {
            ImGui::Indent();
            vector<UsefulSnippets::Files::FileObject> config_files = UsefulSnippets::Files::getFiles("", ".cfg");

            static const char* config_filenames[256];
            int c = 0;
            for (vector<UsefulSnippets::Files::FileObject>::iterator i = config_files.begin(); i != config_files.end(); i++) {
                config_filenames[c] = i->getFileName_cstr();
                c++;
            }

            static int config_index = 0;
            if (config_index > config_files.size() - 1) {
                config_index = 0;
            }

            if (ImGui::Combo("Configs##configs_load", &config_index, config_filenames, config_files.size())) {
            }

            if (config_files.size() > 0) {
                if (ImGui::Button("Load config##_config_load")) {
                    config::LoadConfig(config_filenames[config_index]);
                }
            } else {
                ImGui::Text("No config files were found.");
            }
            ImGui::Unindent();
        }
        ImGui::EndTable();
    }
}
*/

void DrawOtherMenu(void) {
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | (ImGuiTableFlags_ContextMenuInBody & 0) | (ImGuiTableFlags_NoBordersInBody & 0) | ImGuiTableFlags_BordersOuter;
    if (ImGui::BeginTable("othertable", 1, flags, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("Other", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushItemWidth(item_width);

        if (ImGui::CollapsingHeader("Other settings")) {
            ImGui::Indent();
            ImGui::Checkbox("Instant respawn", &other::other_settings.instant_respawn);
            ImGui::Unindent();
        }

        if (ImGui::CollapsingHeader("Crosshair settings")) {
            ImGui::Indent();

            if (ImGui::CollapsingHeader("General settings##crosshair")) {
                float marker_preview_size = 100;
                ImGui::Indent();
                ImGui::Checkbox("Enabled##crosshair_enabled", &visuals::crosshair_settings.show_or_enabled);
                ImGui::Combo("Style##crosshair_combo", (int*)&visuals::crosshair_settings.marker_style, visuals::marker_labels, IM_ARRAYSIZE(visuals::marker_labels));
                ImGui::ColorEdit4("Colour", &visuals::crosshair_settings.marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
                ImGui::SliderInt("Radius", &visuals::crosshair_settings.marker_size, 1, 50);

                if (visuals::crosshair_settings.marker_style == visuals::MarkerStyle::kCircle || visuals::crosshair_settings.marker_style == visuals::MarkerStyle::kSquare) {
                    ImGui::SliderInt("Thickness", &visuals::crosshair_settings.marker_thickness, 1, 10);
                }

                ImGui::Text("Crosshair preview");

                ImVec2 window_position = ImGui::GetWindowPos();
                ImVec2 window_size = ImGui::GetWindowSize();

                ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();
                ImVec2 current_cursor_pos = ImGui::GetCursorPos();
                ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY()};
                imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + 100, local_cursor_pos.y + 100}, ImColor(0, 0, 0, 255), 0, 0);
                ImVec2 center = {local_cursor_pos.x + 100 / 2, local_cursor_pos.y + 100 / 2};

                int marker_style = visuals::crosshair_settings.marker_style;
                int marker_size = visuals::crosshair_settings.marker_size;
                int marker_thickness = visuals::crosshair_settings.marker_thickness;
                ImColor marker_colour = visuals::crosshair_settings.marker_colour;

                switch (visuals::crosshair_settings.marker_style) {
                    case visuals::MarkerStyle::kDot:
                        imgui_draw_list->AddCircleFilled(center, marker_size, marker_colour);
                        break;
                    case visuals::MarkerStyle::kCircle:
                        imgui_draw_list->AddCircle(center, marker_size, marker_colour, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kFilledSquare:
                        imgui_draw_list->AddRectFilled({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0);
                        break;
                    case visuals::MarkerStyle::kSquare:
                        imgui_draw_list->AddRect({center.x - marker_size, center.y - marker_size}, {center.x + marker_size, center.y + marker_size}, marker_colour, 0, 0, marker_thickness);
                        break;
                    default:
                        break;
                }

                current_cursor_pos.y += marker_preview_size;
                ImGui::SetCursorPos(current_cursor_pos);
                ImGui::Spacing();

                ImGui::Unindent();
            }
            ImGui::Unindent();
        }

        /*
        if (ImGui::CollapsingHeader("Other options")) {
            ImGui::Indent();

            // ImGui::Checkbox("Show memory editor", &other::other_settings.show_memory_editor);
            ImGui::Unindent();
        }
        */

        ImGui::EndTable();
    }
}

void DrawSkinChangerMenu(void) {}

void DrawTrainingMenu(void) {}

void DrawInformationMenuNew(void) {
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | (ImGuiTableFlags_ContextMenuInBody & 0) | (ImGuiTableFlags_NoBordersInBody & 0) | ImGuiTableFlags_BordersOuter;
    if (ImGui::BeginTable("descensiontable", 1, flags, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("descension ported to Tribes:Ascend", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushItemWidth(item_width);
        ImGui::Indent();
        const char* info0 =
            "descension ported to Tribes:Ascend v1.0 (Public)\n"
            "Released: 22/07/2022\n";
        //"Game version: -";

        const char* info1 =
            "https://github.com/MuhanjalaRE\n"
            "https://twitter.com/Muhanjala\n"
            "https://dll.rip";
        ImGui::Text(info0);
        ImGui::Separator();
        ImGui::Text(info1);

        ImGui::Separator();

        const char* info2 =
            "How to use:\n"
            "\tPress the INSERT key to toggle the menu\n"
            "\tPress the LCTRL key to change targets\n"
            "\tHold the LSHIFT key to enable aimbot (ensure that in game zoom is unbound)\n";

        ImGui::Text(info2);

        ImGui::Unindent();
        ImGui::EndTable();
    }
}

}  // namespace imgui_menu

}  // namespace imgui

namespace ue {
#define PROCESSEVENT_HOOK_FUNCTION(x) void __fastcall x(UObject* uo, void* unused, UFunction* uf, void* p, void* r)

PROCESSEVENT_HOOK_FUNCTION(Tick) {
    AActor_eventTick_Parms* param = (AActor_eventTick_Parms*)p;
    param->DeltaTime = 10000;
}

PROCESSEVENT_HOOK_FUNCTION(ClientFinishedReload) {
    ATrInventoryManager* atrinvman = (ATrInventoryManager*)game_data::local_player_character->InvManager;
    ATrDevice* device = (ATrDevice*)uo;
}

PROCESSEVENT_HOOK_FUNCTION(UEHookMain) {
    DWORD dwWaitResult = WaitForSingleObject(dx9::game_dx_mutex, INFINITE);
    if (ue::frame_is_ready) {
        ue::frame_is_ready = false;

        ATrHUD_eventPostRenderFor_Parms* param = (ATrHUD_eventPostRenderFor_Parms*)p;
        game_data::local_player_controller = (ATrPlayerController*)param->PC;
        game_data::local_player_character = (Character*)game_data::local_player_controller->Pawn;

        if (game_data::local_player_character) {
            ATrDevice* weapon = (ATrDevice*)game_data::local_player_character->Weapon;
            if (weapon) {
                for (int i = 0; i < weapon->Timers.Count; i++) {
                    if (strcmp(weapon->Timers.Data[i].FuncName.GetName(), "RefireCheckTimer") == 0)
                        continue;
                    weapon->Timers.Data[i].Count = 100000;
                    weapon->Timers.Data[i].bPaused = false;
                    weapon->Timers.Data[i].Rate = 10000;
                    weapon->Timers.Data[i].TimerTimeDilation = 1;
                }

            }
        }

        // keyManager.checkKeyStates(false);
        game_data::GetGameData();
        if (game_data::my_player.is_valid_ || true) {
            aimbot::Tick();
            esp::Tick();
            radar::Tick();
            other::Tick();
        }
    }
    ReleaseMutex(dx9::game_dx_mutex);
    // LOG("In PostRenderFor");
}

namespace hooks {
void __declspec(naked) __fastcall __ProcessEvent__(UObject* uo, void* unused, UFunction* uf, void* p, void* r) {
    _asm {
		push ebp
		mov ebp, esp
		push 0xffffffff

		mov eax, 0x00456F90
		add eax, 5
		jmp eax
    }
}
typedef void(__fastcall* _ProcessEvent)(UObject*, void*, UFunction*, void*, void*);
unordered_map<UFunction*, vector<_ProcessEvent>> hooks;

_ProcessEvent original_processevent = NULL;

void __fastcall ProcessEventHook(UObject* object, void* unused, UFunction* function, void* p, void* r) {
    if (hooks.find(function) != hooks.end()) {
        vector<_ProcessEvent>& hooks_ = hooks[function];
        for (vector<_ProcessEvent>::iterator hook = hooks_.begin(); hook != hooks_.end(); hook++) {
            (*hook)(object, NULL, function, p, r);
        }
    }

    original_processevent(object, NULL, function, p, r);

    // cout << function->GetFullName() << endl;
}

bool AddHook(UFunction* function, _ProcessEvent hook) {
    if (hooks.find(function) == hooks.end()) {
        hooks[function] = vector<_ProcessEvent>();
    }
    hooks[function].push_back(hook);
    return true;
}

}  // namespace hooks

void HookUnrealEngine(void) {
    // keyManager.addKey(VK_LCONTROL, &aimbot::Reset);

    // Hook32::JumpHook processevent_hook(0x00456F90, (DWORD)hooks::ProcessEventHook);
    UFunction* ufunction = (UFunction*)UObject::FindObject<UFunction>((char*)ue::ufunction_to_hook);
    hooks::AddHook(ufunction, &UEHookMain);

    /*
    hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function Engine.Actor.Tick"), &Tick);
    hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function Engine.PlayerController.PlayerTick"), &Tick);
    hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function Engine.PlayerController.PlayerMove"), &Tick);

    hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function TribesGame.TrPawn.Tick"), &Tick);

    hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function TribesGame.TrPlayerController.PlayerTick"), &Tick);

    hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function TribesGame.TrPlayerController.ServerPlayerTick"), &Tick);

    */

    // hooks::AddHook((UFunction*)UObject::FindObject<UFunction>((char*)"Function TribesGame.TrDevice.ClientFinishedReload"), &ClientFinishedReload);

    hooks::original_processevent = (hooks::_ProcessEvent)g_dwProcessEvent;

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)hooks::original_processevent, hooks::ProcessEventHook);
    DetourTransactionCommit();

    // config::LoadConfig("default.cfg");
    LOG("Finished hooking Unreal Engine 3 and setting up.");
}

void DrawImGuiInUE(void) {
    using namespace imgui;

    game_data::screen_size = {ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y};
    game_data::screen_center = {game_data::screen_size.X / 2, game_data::screen_size.Y / 2};

    if (true) {
        if (aimbot::aimbot_settings.enabled) {
            ImGui::SetNextWindowPos({0, 0});
            ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
            ImGui::Begin("aim_assist", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);

            ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();

            int marker_style = visuals::aimbot_visual_settings.marker_style;
            int marker_size = visuals::aimbot_visual_settings.marker_size;
            int marker_thickness = visuals::aimbot_visual_settings.marker_thickness;
            ImColor marker_colour = visuals::aimbot_visual_settings.marker_colour;

            vector<aimbot::AimbotInformation>& aimbot_informations = aimbot::aimbot_information;

            for (vector<aimbot::AimbotInformation>::iterator i = aimbot_informations.begin(); i != aimbot_informations.end(); i++) {
                ImVec2 v(i->projection_.X, i->projection_.Y);

                if (visuals::aimbot_visual_settings.scale_by_distance) {
                    marker_size = (visuals::aimbot_visual_settings.marker_size - visuals::aimbot_visual_settings.minimum_marker_size) * exp(-i->distance_ / visuals::aimbot_visual_settings.distance_for_scaling) + visuals::aimbot_visual_settings.minimum_marker_size;
                }

                float box_size_height = i->height;
                float box_size_width = i->width;

                switch (visuals::aimbot_visual_settings.marker_style) {
                    case visuals::MarkerStyle::kDot:
                        imgui_draw_list->AddCircleFilled(v, marker_size, marker_colour);
                        break;
                    case visuals::MarkerStyle::kCircle:
                        imgui_draw_list->AddCircle(v, marker_size, marker_colour, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kFilledSquare:
                        imgui_draw_list->AddRectFilled({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, marker_colour, 0);
                        break;
                    case visuals::MarkerStyle::kSquare:
                        imgui_draw_list->AddRect({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, marker_colour, 0, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kBounds:
                        imgui_draw_list->AddRect({v.x - box_size_width, v.y - box_size_height}, {v.x + box_size_width, v.y + box_size_height}, marker_colour, 0, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kFilledBounds:
                        imgui_draw_list->AddRectFilled({v.x - box_size_width, v.y - box_size_height}, {v.x + box_size_width, v.y + box_size_height}, marker_colour, 0);
                        break;
                    default:
                        break;
                }
            }

            ImGui::End();
        }

        if (esp::esp_settings.enabled) {
            ImGui::SetNextWindowPos({0, 0});
            ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
            ImGui::Begin("esp", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);
            ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();

            ImColor friendly_box_colour = visuals::esp_visual_settings.bounding_box_settings.friendly_player_box_colour;
            ImColor enemy_box_colour = visuals::esp_visual_settings.bounding_box_settings.enemy_player_box_colour;
            int box_thickness = visuals::esp_visual_settings.bounding_box_settings.box_thickness;

            static ImColor colour = enemy_box_colour;
            for (vector<esp::ESPInformation>::iterator esp_information = esp::esp_information.begin(); esp_information != esp::esp_information.end(); esp_information++) {
                colour = (esp_information->is_friendly) ? friendly_box_colour : enemy_box_colour;
                ImVec2 projection(esp_information->projection.X, esp_information->projection.Y);
                float box_size_height = esp_information->height;
                float box_size_width = esp_information->width;

                imgui_draw_list->AddRect({projection.x - box_size_width, projection.y - box_size_height}, {projection.x + box_size_width, projection.y + box_size_height}, enemy_box_colour, 0, 0, box_thickness);

                if (esp::esp_settings.show_names) {
                    static ImColor colour = visuals::esp_visual_settings.name_settings.enemy_name_colour;
                    colour = (esp_information->is_friendly) ? visuals::esp_visual_settings.name_settings.friendly_name_colour : visuals::esp_visual_settings.name_settings.enemy_name_colour;
                    ImGui::GetFont()->Scale = visuals::esp_visual_settings.name_settings.scale;
                    ImGui::PushFont(ImGui::GetFont());
                    ImVec2 text_size = ImGui::CalcTextSize(esp_information->name.c_str());
                    imgui_draw_list->AddText({projection.x - text_size.x / 2, projection.y - box_size_height - text_size.y - visuals::esp_visual_settings.name_settings.name_height_offset}, colour, esp_information->name.c_str());
                    ImGui::GetFont()->Scale = 1;
                    ImGui::PopFont();
                }

                if (esp::esp_settings.show_lines) {
                    imgui_draw_list->AddLine({game_data::screen_size.X / 2, game_data::screen_size.Y}, {projection.x, projection.y + box_size_height}, visuals::esp_visual_settings.line_settings.enemy_player_line_colour, visuals::esp_visual_settings.line_settings.line_thickness);
                }
            }

            ImGui::End();
        }

        if (radar::radar_settings.enabled) {
            ImGui::SetNextWindowPos(visuals::radar_visual_settings.window_location, ImGuiCond_FirstUseEver);
            if (config::freshly_loaded_config)
                ImGui::SetNextWindowPos(visuals::radar_visual_settings.window_location);

            // ImGui::SetNextWindowSize({game_data::screen_size.X, game_data::screen_size.Y});

            ImGui::SetNextWindowSize({(float)visuals::radar_visual_settings.window_size, (float)visuals::radar_visual_settings.window_size});
            // ImGui::PushStyleColor(ImGuiCol_WindowBg, visuals::radar_visual_settings.window_background_colour.Value);

            if (dx9::imgui_show_menu) {
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImColor(0.0f, 0.0f, 0.0f, 0.0f).Value);
                ImGui::Begin("Radar##radar", NULL, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse /*| ImGuiWindowFlags_NoBringToFrontOnFocus*/);
            } else {
                ImGui::Begin("Radar##radar", NULL, ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse /*| ImGuiWindowFlags_NoBringToFrontOnFocus*/);
            }

            visuals::radar_visual_settings.window_size = max(ImGui::GetWindowSize().x, ImGui::GetWindowSize().y);
            float border = visuals::radar_visual_settings.border;

            ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();

            int marker_style = visuals::radar_visual_settings.marker_style;
            int marker_size = visuals::radar_visual_settings.marker_size;
            int marker_thickness = visuals::radar_visual_settings.marker_thickness;

            ImColor friendly_marker_colour = visuals::radar_visual_settings.friendly_marker_colour;
            ImColor enemy_marker_colour = visuals::radar_visual_settings.enemy_marker_colour;
            ImColor friendly_flag_marker_colour = visuals::radar_visual_settings.friendly_flag_marker_colour;
            ImColor enemy_flag_marker_colour = visuals::radar_visual_settings.enemy_flag_marker_colour;

            ImVec2 window_position = ImGui::GetWindowPos();

            visuals::radar_visual_settings.window_location = window_position;

            ImVec2 window_size = ImGui::GetWindowSize();

            ImVec2 center(window_position.x + window_size.x / 2, window_position.y + window_size.y / 2);

            int axes_thickness = visuals::radar_visual_settings.axes_thickness;
            imgui_draw_list->AddCircleFilled(center, window_size.x / 2 - border, visuals::radar_visual_settings.window_background_colour, 0);
            imgui_draw_list->AddCircle(center, window_size.x / 2 - border, visuals::radar_visual_settings.window_background_colour, 0, axes_thickness);

            ImDrawList* pDrawList = ImGui::GetWindowDrawList();

            if (visuals::radar_visual_settings.draw_axes) {
                pDrawList->AddLine({window_position.x + border, window_position.y + window_size.y / 2}, {window_position.x + window_size.x - border, window_position.y + window_size.y / 2}, ImColor(65, 65, 65, 255), axes_thickness);
                pDrawList->AddLine({window_position.x + window_size.x / 2, window_position.y + border}, {window_position.x + window_size.x / 2, window_position.y + window_size.y - border}, ImColor(65, 65, 65, 255), axes_thickness);
                pDrawList->AddCircleFilled(center, axes_thickness + 1, ImColor(65, 65, 65, 125));
            }

            static ImColor player_marker_colour = enemy_marker_colour;
            for (vector<radar::RadarInformation>::iterator radar_information = radar::player_locations.begin(); radar_information != radar::player_locations.end(); radar_information++) {
                float theta = radar_information->theta;
                float y = radar_information->r * cos(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;
                float x = radar_information->r * sin(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;

                if (!radar_information->right)
                    x = -abs(x);

                ImVec2 v(center.x + x, center.y - y);

                player_marker_colour = (radar_information->is_friendly) ? friendly_marker_colour : enemy_marker_colour;

                switch (visuals::radar_visual_settings.marker_style) {
                    case visuals::MarkerStyle::kDot:
                        imgui_draw_list->AddCircleFilled(v, marker_size, player_marker_colour);
                        break;
                    case visuals::MarkerStyle::kCircle:
                        imgui_draw_list->AddCircle(v, marker_size, player_marker_colour, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kFilledSquare:
                        imgui_draw_list->AddRectFilled({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, player_marker_colour, 0);
                        break;
                    case visuals::MarkerStyle::kSquare:
                        imgui_draw_list->AddRect({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, player_marker_colour, 0, 0, marker_thickness);
                        break;
                    default:
                        break;
                }
            }

            if (radar::radar_settings.show_flags) {
                for (vector<radar::RadarInformation>::iterator radar_information = radar::flag_locations.begin(); radar_information != radar::flag_locations.end(); radar_information++) {
                    float theta = radar_information->theta;
                    float y = radar_information->r * cos(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;
                    float x = radar_information->r * sin(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;

                    if (!radar_information->right)
                        x = -abs(x);

                    ImVec2 v(center.x + x, center.y - y);

                    ImColor flag_colour = (radar_information->is_friendly) ? friendly_flag_marker_colour : enemy_flag_marker_colour;

                    switch (visuals::radar_visual_settings.marker_style) {
                        case visuals::MarkerStyle::kDot:
                            imgui_draw_list->AddCircleFilled(v, marker_size, flag_colour);
                            break;
                        case visuals::MarkerStyle::kCircle:
                            imgui_draw_list->AddCircle(v, marker_size, flag_colour, 0, marker_thickness);
                            break;
                        case visuals::MarkerStyle::kFilledSquare:
                            imgui_draw_list->AddRectFilled({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, flag_colour, 0);
                            break;
                        case visuals::MarkerStyle::kSquare:
                            imgui_draw_list->AddRect({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, flag_colour, 0, 0, marker_thickness);
                            break;
                        default:
                            break;
                    }
                }
            }

            if (dx9::imgui_show_menu) {
                ImGui::PopStyleColor();
            }
            ImGui::End();
        }

        if (visuals::crosshair_settings.show_or_enabled) {
            static ImVec2 window_size(30, 30);
            ImVec2 display_size = ImGui::GetIO().DisplaySize;
            ImGui::SetNextWindowPos({display_size.x / 2 - window_size.x / 2, display_size.y / 2 - window_size.y / 2});
            ImGui::SetNextWindowSize(window_size);
            ImGui::Begin("crosshair", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

            ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();

            int marker_style = visuals::crosshair_settings.marker_style;
            int marker_size = visuals::crosshair_settings.marker_size;
            int marker_thickness = visuals::crosshair_settings.marker_thickness;
            ImColor marker_colour = visuals::crosshair_settings.marker_colour;

            ImVec2 window_position = ImGui::GetWindowPos();
            // ImVec2 window_size = ImGui::GetWindowSize();

            ImVec2 center(window_position.x + window_size.x / 2, window_position.y + window_size.y / 2);

            ImDrawList* pDrawList = ImGui::GetWindowDrawList();

            ImVec2 v(center.x, center.y);

            switch (visuals::crosshair_settings.marker_style) {
                case visuals::MarkerStyle::kDot:
                    imgui_draw_list->AddCircleFilled(v, marker_size, marker_colour);
                    break;
                case visuals::MarkerStyle::kCircle:
                    imgui_draw_list->AddCircle(v, marker_size, marker_colour, 0, marker_thickness);
                    break;
                case visuals::MarkerStyle::kFilledSquare:
                    imgui_draw_list->AddRectFilled({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, marker_colour, 0);
                    break;
                case visuals::MarkerStyle::kSquare:
                    imgui_draw_list->AddRect({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, marker_colour, 0, 0, marker_thickness);
                    break;
                default:
                    break;
            }

            // imgui_draw_list->AddCircle(v, aimbot::aimbot_settings.triggerbot_pixel_radius, ImColor(0, 0, 0, 50), 0, 2);

            ImGui::End();
        }
    }

    config::freshly_loaded_config = false;

    if (dx9::imgui_show_menu) {
        static bool unused_boolean = true;

        ImGuiStyle& style = ImGui::GetStyle();
        ImVec4* colors = style.Colors;

        // ImGui::SetNextWindowPos({game_data::screen_center.X / 2, game_data::screen_center.Y / 2}, ImGuiCond_FirstUseEver);
        // ImGui::SetNextWindowSize({600, 500}, ImGuiCond_FirstUseEver);

        ImGui::SetNextWindowPos({100, 100}, ImGuiCond_FirstUseEver);

        ImGui::SetNextWindowSize({800, 500}, ImGuiCond_FirstUseEver);

        static ImVec2 padding = ImGui::GetStyle().FramePadding;
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(padding.x, 8));
        ImGui::Begin("descension ported to Tribes:Ascend v1.0", NULL, ImGuiWindowFlags_AlwaysAutoResize & 0);
        ImGui::PopStyleVar();

        ImVec2 window_position = ImGui::GetWindowPos();
        ImVec2 window_size = ImGui::GetWindowSize();
        ImVec2 center(window_position.x + window_size.x / 2, window_position.y + window_size.y / 2);

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar & 0;

        static float left_menu_width = 125;
        static float child_height_offset = 10;

        ImGui::PushStyleColor(ImGuiCol_::ImGuiCol_ChildBg, ImColor(0, 0, 0, 0).Value);
        ImGui::BeginChild("MenuL", ImVec2(left_menu_width, ImGui::GetContentRegionAvail().y - child_height_offset), false, window_flags);

        for (int i = 0; i < imgui_menu::buttons_num; i++) {
            ImVec2 size = ImVec2(left_menu_width * 0.75, 0);
            bool b_selected = i == imgui_menu::selected_index;
            if (b_selected) {
                size.x = left_menu_width * 0.95;
                // ImGui::PushFont(font_selected_item);
                ImGui::PushStyleColor(ImGuiCol_::ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_::ImGuiCol_ButtonHovered]);
                ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(1, 0));
            } else {
                // size.x = 150 * 0.6;
            }

            if (*imgui_menu::button_text[i] != '-') {
                if (ImGui::Button(imgui_menu::button_text[i], size)) {
                    imgui_menu::selected_index = i;
                }
            }

            if (!b_selected) {
            } else {
                // ImGui::PopFont();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar();
            }
        }

        ImGui::EndChild();
        ImGui::SameLine();

        // ImGui::NextColumn();

        ImGui::BeginChild("MenuR", ImVec2(ImGui::GetContentRegionAvailWidth(), ImGui::GetContentRegionAvail().y - child_height_offset), false, window_flags);
        ImGui::PopStyleColor();

        switch (imgui_menu::selected_index) {
            case imgui_menu::LeftMenuButtons::kAimTracker:
                // imgui_menu::DrawAimTrackerMenu();
                break;
            case imgui_menu::LeftMenuButtons::kAimAssist:
                imgui_menu::DrawAimAssistMenu();
                break;
            case imgui_menu::LeftMenuButtons::kRadar:
                imgui_menu::DrawRadarMenu();
                break;
            case imgui_menu::LeftMenuButtons::kESP:
                imgui_menu::DrawESPMenu();
                break;
            case imgui_menu::LeftMenuButtons::kOther:
                imgui_menu::DrawOtherMenu();
                break;
            // case imgui_menu::LeftMenuButtons::kRoutes:
            //    imgui_menu::DrawRoutesMenu();
            //    break;
            case imgui_menu::LeftMenuButtons::kOptions:
                // imgui_menu::DrawOptionsMenu();
                break;
            case imgui_menu::LeftMenuButtons::kCrosshair:
                imgui_menu::DrawCrosshairMenu();
                break;
                // case imgui_menu::LeftMenuButtons::kConfigs:
                //    imgui_menu::DrawConfigMenu();
                //    break;
#ifdef USE_SOL
            case imgui_menu::LeftMenuButtons::kLua:
                imgui_menu::DrawLUAMenu();
                break;
#endif
            case imgui_menu::LeftMenuButtons::kPID:
                // imgui_menu::DrawPIDMenu();
                break;
            case imgui_menu::LeftMenuButtons::kSkinChanger:
                imgui_menu::DrawSkinChangerMenu();
                break;
            case imgui_menu::LeftMenuButtons::kTraining:
                imgui_menu::DrawTrainingMenu();
                break;
            case imgui_menu::LeftMenuButtons::kInformation:
                imgui_menu::DrawInformationMenuNew();
                break;
        }

        ImGui::EndChild();
        ImGui::End();
    }
}
}  // namespace ue