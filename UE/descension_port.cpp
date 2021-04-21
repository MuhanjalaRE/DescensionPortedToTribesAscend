#include "DX.h"
#include "ue.h"

#include "imgui.h"
#include "imgui_impl_dx9.h"
#include "imgui_impl_win32.h"

#include "Other/Keys/Keys.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <utility>

//#define USE_SOL
#ifdef USE_SOL
#include <sol/sol.hpp>
#endif

#include "SDK.h"

#include "detours.h"

#include "Hook.h"
#include "log.h"

#include <unordered_map>

using namespace std;
using namespace UE_Utilities;

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
static enum MarkerStyle { kDot, kCircle, kFilledSquare, kSquare, kFilledRectangle, kRectangle };
static const char* marker_labels[] = {"Dot", "Circle", "Filled square", "Square"};
// static const char* marker_labels[] = {"Dot", "Circle", "Filled square", "Square", "Filled rectangle", "Rectangle"};

static struct AimbotVisualSettings {
    MarkerStyle marker_style = MarkerStyle::kSquare;
    int marker_size = 6;
    int marker_thickness = 2;
    ImColor marker_colour = {255, 255, 0, 200};
    int marker_size_width = 5;
    int marker_size_height = 5;
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
        ImColor enemy_name_colour = {255, 255, 255, 1 * 255};
        ImColor friendly_name_colour = {0, 255, 0, 1 * 255};
    } name_settings;

} esp_visual_settings;

static struct CrosshairSettings {
    bool show_or_enabled = true;
    MarkerStyle marker_style = MarkerStyle::kDot;
    int marker_size = 3;
    int marker_thickness = 1;
    ImColor marker_colour = {255, 0, 255, 255};
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
    return (character && character->PlayerReplicationInfo && character->Health > 0);
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
    static const float Const_RadToUnrRot = 10430.3783504704527;
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
        float ping_;
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
    bool PredictAimAtTarget_DicksuckingLord(WorldObject* target_object, Vector* output_vector = NULL, Vector offset = Vector());
    Vector FactorInGravity(Vector target_location, Vector target_velocity, float gravity, float time, Vector offset = Vector());
} my_weapon_object;

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

    ping_prediction = prediction = (target_location + (target_velocity * full_time * 1) + (target_acceleration * pow(dt[i], 2) * 0.5) - (weapon_parameters_.use_inheritance ? (owner_velocity * (weapon_parameters_.inheritence_ * full_time)) : Vector()));

    *output_vector = ping_prediction;

    if (weapon_type_ == WeaponType::kProjectileArching) {
        // ping_prediction = FactorInGravity(target_location, target_velocity, 1500, full_time, offset);
        return false;
        *output_vector = ping_prediction;
    }

    return true;
}
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
    local_player_character = (Character*)local_player_controller->Pawn;
    if (!validate::IsValid(local_player_character)) {
        return;
    }

    bool my_player_character_found = false;

    APawn* pawns = local_player_controller->Pawn->WorldInfo->PawnList;
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
        return;
    }
}

void GetWeapon(void) {
    if (!my_player.is_valid_) {
        return;
    }

    ATrDevice* weapon = (ATrDevice*)my_player.character_->Weapon;
    if (weapon) {
        game_data::my_player.weapon_ = game_data::Weapon::found;
        if (weapon->bInstantHit && !weapon->IsA(ATrDevice_ConstantFire::StaticClass())) {
            abstraction::my_weapon_object.SetWeaponType(abstraction::WeaponObject::WeaponType::kHitscan);
        } else {
            static abstraction::WeaponObject::WeaponParameters* weapon_parameters = abstraction::my_weapon_object.GetWeaponParameters();
            static abstraction::WeaponObject::WeaponAimbotParameters* aimbot_parameters = abstraction::my_weapon_object.GetAimbotParameters();

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

void GetGameData(void) {
    game_data.Reset();
    GetPlayers();
    GetWeapon();
}

}  // namespace game_data

namespace game_functions {
bool InLineOfSight(AActor* actor) {
    return game_data::local_player_character->LineOfSightTo(actor);
}

FVector2D Project(FVector location) {
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

namespace aimbot {

// Overshooting means the weapon bullet speed is too low
// Undershooting means the weapon bullet speed is too high

enum AimbotMode { kClosestDistance, kClosestXhair };
static const char* mode_labels[] = {"Closest distance", "Closest to xhair"};
static bool enabled = true;

static struct AimbotSettings {
    AimbotMode aimbot_mode = AimbotMode::kClosestXhair;

    bool enabled = true;  // enabling really just enables aimassist, this isnt really an aimbot
    bool use_custom_ping = true;
    bool auto_aim = false;        // this enables the aimbot
    bool target_everyone = true;  // if we want to do prediction on every single player

    float tempest_ping_in_ms = 0;   //-90
    float chaingun_ping_in_ms = 0;  //-50
    float grenadelauncher_ping_in_ms = 0;
    float plasmagun_ping_in_ms = 0;
    float blaster_ping_in_ms = 0;

    int maximum_iterations = 10;
    int epsilon = 0.05;

    bool stay_locked_to_target = true;
    bool auto_lock_to_new_target = true;

    float aimbot_horizontal_fov_angle = 90;         // 30;
    float aimbot_horizontal_fov_angle_cos = 0;      // 0.86602540378;
    float aimbot_horizontal_fov_angle_cos_sqr = 0;  // 0.75;

    bool friendly_fire = false;
    bool need_line_of_sight = true;

    int aimbot_poll_frequency = 300;

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

} aimbot_settings;

static Timer aimbot_poll_timer(aimbot_settings.aimbot_poll_frequency);

static game_data::information::Player target_player;

vector<FVector2D> projections_of_predictions;

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
        if (!validate::IsValid(target_player.character_)) {
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

void Tick(void) {
    if (!aimbot_settings.enabled /*|| !enabled*/ || !aimbot_poll_timer.isReady())
        return;

    // SetupWeapon();

    projections_of_predictions.clear();
    projections_of_predictions_coloured.clear();

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
        muzzle_offset = game_data::my_player.character_->Weapon->FireOffset;
    }

    if (!aimbot_settings.target_everyone) {
        if (enabled && FindTarget() /*&& target_player.character_*/) {
            player_world_object.SetLocation(target_player.location_);
            player_world_object.SetVelocity(target_player.velocity_);

            bool result = abstraction::my_weapon_object.PredictAimAtTarget(&player_world_object, &prediction, muzzle_offset);

            if (result) {
                FVector2D projection = game_functions::Project(prediction);
                projections_of_predictions.push_back(projection);

                if (aimbot_settings.auto_aim) {
                    prediction.Z -= aimbot_settings.aimbot_offset.Z;
                    FRotator aim_rotator = math::VectorToRotator(prediction - game_data::my_player.location_);
                    FRotator& aim_rotator_reference = aim_rotator;
                    game_data::local_player_controller->SetRotation(aim_rotator_reference);
                }
            }
        }
    } else {
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

            bool result = abstraction::my_weapon_object.PredictAimAtTarget(&player_world_object, &prediction, muzzle_offset);

            if (result) {
                FVector2D projection = game_functions::Project(prediction);
                projections_of_predictions.push_back(projection);
                // math::PrintVector(prediction, "Prediction");
            }
        }
    }
}

}  // namespace aimbot

namespace esp {

static struct ESPSettings {
    bool enabled = true;
    int poll_frequency = 60;
    bool show_friendlies = false;
    int player_height = 100;
    int player_width = 0;
    float width_to_height_ratio = 0.5;
    bool show_lines = false;
    bool show_names = false;

} esp_settings;

static Timer get_esp_data_timer(esp_settings.poll_frequency);

struct ESPInformation {
    FVector2D projection;  // center
    float height;          // height for box/rectangle
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
        player->location_.Z += esp_settings.player_height;  // this is HALF the height in reality
        FVector2D head_projection = game_functions::Project(player->location_);
        player->location_.Z -= esp_settings.player_height;  // this is HALF the height in reality
        float height = abs(head_projection.Y - center_projection.Y);

        string name;
        if (esp_settings.show_names) {
            // name = player->player_state_->PlayerName.ToString();
        }

        esp_information.push_back({center_projection, height, same_team, name});
    }
}
}  // namespace esp

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

namespace imgui {
namespace imgui_menu {
enum LeftMenuButtons { kAimAssist, kAimbot, kESP, kRadar, kOther, kAimTracker, kRoutes, kOptions, kCrosshair, kSkinChanger, kConfigs, kCredits, kLua, kBindings, kPID, kTraining };
static const char* button_text[] = {"Aim assist", "-", "ESP", "Radar", "Other", "-", "Routes", "-", "Crosshair", "Shop", "Configs", "-", "-", "-", "-", "Training"};
// static const char* button_text[] = {"-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "Scripting", "-", "-"};
static const int buttons_num = sizeof(button_text) / sizeof(char*);
static int selected_index = LeftMenuButtons::kAimAssist;

void DrawAimAssistMenu(void) {
    // static int marker_style = visuals::aimbot_visual_settings.marker_style;
    // static int marker_size = visuals::aimbot_visual_settings.marker_size;
    // static int marker_thickness = visuals::aimbot_visual_settings.marker_thickness;
    // static ImColor marker_colour = visuals::aimbot_visual_settings.marker_colour;

    int right_child_width = 200;

    ImGui::BeginGroup();
    ImGui::BeginChild("left_settings##left_settings", ImVec2(ImGui::GetWindowContentRegionWidth() > right_child_width ? ImGui::GetWindowContentRegionWidth() - right_child_width : ImGui::GetWindowContentRegionWidth(), 0));

    ImGui::Text("Aimbot Settings");
    // ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.75f, 0, 0, 255));
    ImGui::Separator();
    // ImGui::PopStyleColor();

    if (ImGui::Checkbox("Enable aim assist", &aimbot::aimbot_settings.enabled)) {
    }

    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Enable aim assist. This draw predictions on the screen.");
    }

    ImGui::PushItemWidth(100);
    ImGui::Combo("Mode##aim_assist_mode_combo", (int*)&aimbot::aimbot_settings.aimbot_mode, aimbot::mode_labels, IM_ARRAYSIZE(aimbot::mode_labels));
    ImGui::PopItemWidth();

    if (aimbot::aimbot_settings.aimbot_mode == aimbot::AimbotMode::kClosestXhair) {
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("Horizontal FOV", &aimbot::aimbot_settings.aimbot_horizontal_fov_angle, 1, 90);
        ImGui::PopItemWidth();
        aimbot::aimbot_settings.aimbot_horizontal_fov_angle_cos = cos(aimbot::aimbot_settings.aimbot_horizontal_fov_angle * PI / 180.0);
        aimbot::aimbot_settings.aimbot_horizontal_fov_angle_cos_sqr = pow(aimbot::aimbot_settings.aimbot_horizontal_fov_angle_cos, 2);
    }

    if (!aimbot::aimbot_settings.target_everyone) {
        ImGui::Checkbox("Stay locked on to target", &aimbot::aimbot_settings.stay_locked_to_target);
        ImGui::Checkbox("Auto lock to new target", &aimbot::aimbot_settings.auto_lock_to_new_target);
    }

    ImGui::PushItemWidth(100);
    if (ImGui::SliderInt("Poll rate (Hz)", &aimbot::aimbot_settings.aimbot_poll_frequency, 1, 300)) {
        aimbot::aimbot_poll_timer.SetFrequency(aimbot::aimbot_settings.aimbot_poll_frequency);
    }
    ImGui::PopItemWidth();

    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(2 * 0.05, 2 * 0.05, 2 * 0.05, 2 * 0.1));
    ImGui::Separator();
    ImGui::PopStyleColor();

    ImGui::Text("Weapon settings");
    // ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.75f, 0, 0, 255));
    ImGui::Separator();
    // ImGui::PopStyleColor();

    // ImGui::Checkbox("Use inheritance", &game_data::my_player_object.GetWeapon()->GetAimbotParameters()->use_inheritance);
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);
    ImGui::SliderFloat("Tempest ping", &aimbot::aimbot_settings.tempest_ping_in_ms, -300, 300);
    ImGui::SliderFloat("Chain gun ping", &aimbot::aimbot_settings.chaingun_ping_in_ms, -300, 300);
    ImGui::SliderFloat("Grenade launcher ping", &aimbot::aimbot_settings.grenadelauncher_ping_in_ms, -300, 300);
    ImGui::SliderFloat("Plasma gun ping", &aimbot::aimbot_settings.plasmagun_ping_in_ms, -300, 300);
    ImGui::SliderFloat("Blaster ping", &aimbot::aimbot_settings.blaster_ping_in_ms, -300, 300);
    ImGui::SliderFloat("Self compensation ping", &aimbot::aimbot_settings.self_compensation_time_in_ms, -300, 300);
    // ImGui::SliderFloat("Tempest ping", &aimbot::aimbot_settings.tempest_ping_in_ms, -200, 200);
    ImGui::PopItemWidth();

    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(2 * 0.05, 2 * 0.05, 2 * 0.05, 2 * 0.1));
    ImGui::Separator();
    ImGui::PopStyleColor();

    ImGui::Text("Target settings");
    ImGui::Separator();
    ImGui::Checkbox("Friendly fire", &aimbot::aimbot_settings.friendly_fire);
    ImGui::Checkbox("Target everyone", &aimbot::aimbot_settings.target_everyone);
    ImGui::Checkbox("Need line of sight", &aimbot::aimbot_settings.need_line_of_sight);

    /*

    ImGui::Separator();
    ImGui::Checkbox("Triggerbot", &aimbot::aimbot_settings.use_triggerbot);
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);
    ImGui::SliderFloat("Triggerbot pixel radius", &aimbot::aimbot_settings.triggerbot_pixel_radius, 1, game_data::screen_center.X);
    ImGui::PopItemWidth();

    ImGui::Separator();
    ImGui::Checkbox("Auto aim", &aimbot::aimbot_settings.auto_aim);

    ImGui::Separator();
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);
    ImGui::SliderFloat("Aimbot offset X", &aimbot::aimbot_settings.aimbot_offset.X, -300, 300);
    ImGui::SliderFloat("Aimbot offset Y", &aimbot::aimbot_settings.aimbot_offset.Y, -300, 300);
    ImGui::SliderFloat("Aimbot offset Z", &aimbot::aimbot_settings.aimbot_offset.Z, -300, 300);
    ImGui::PopItemWidth();

    ImGui::Separator();
    ImGui::Checkbox("Use weighting", &aimbot::aimbot_settings.use_weighting);
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);
    ImGui::SliderInt("Client weight", &aimbot::aimbot_settings.client_weight, 0, 10);
    ImGui::SliderInt("Prediction weight", &aimbot::aimbot_settings.prediction_weight, 0, 10);
    */

    if (!aimbot::aimbot_settings.target_everyone) {
        ImGui::Separator();
        ImGui::Checkbox("Auto aim", &aimbot::aimbot_settings.auto_aim);

        if (aimbot::aimbot_settings.auto_aim) {
            ImGui::PushItemWidth(100);
            ImGui::SliderFloat("Aimbot offset X", &aimbot::aimbot_settings.aimbot_offset.X, -300, 300);
            ImGui::SliderFloat("Aimbot offset Y", &aimbot::aimbot_settings.aimbot_offset.Y, -300, 300);
            ImGui::SliderFloat("Aimbot offset Z", &aimbot::aimbot_settings.aimbot_offset.Z, -300, 300);
            ImGui::PopItemWidth();
        }
    }

    /*

    ImGui::Text("Experimental shit");
    ImGui::Separator();
    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("XY unit velocity scalar (tempest)", &aimbot::xy_velocity_scalar, -100, 100);
    ImGui::SliderFloat("Z velocity scalar (tempest)", &aimbot::z_velocity_scalar, -1, 1);

    ImGui::Checkbox("Use muzzle location", &aimbot::use_muzzle_location);
    ImGui::SliderFloat("Muzzle scalar", &aimbot::muzzle_scalar, -100, 100);

    ImGui::PopItemWidth();

    */

    ImGui::EndChild();
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::SetCursorPosX({ImGui::GetCursorPos().x + 10});
    ImGui::BeginGroup();
    ImGui::BeginChild("right_settings##right_settings", ImVec2(right_child_width, 0));

    ImGui::Text("Marker settings");
    ImGui::Separator();
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);

    ImGui::Combo("Style##aim_assist_combo", (int*)&visuals::aimbot_visual_settings.marker_style, visuals::marker_labels, IM_ARRAYSIZE(visuals::marker_labels));
    ImGui::ColorEdit4("Colour", &visuals::aimbot_visual_settings.marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::SliderInt("Radius", &visuals::aimbot_visual_settings.marker_size, 1, 10);

    if (visuals::aimbot_visual_settings.marker_style == visuals::MarkerStyle::kCircle || visuals::aimbot_visual_settings.marker_style == visuals::MarkerStyle::kSquare) {
        ImGui::SliderInt("Thickness", &visuals::aimbot_visual_settings.marker_thickness, 1, 10);
    }

    ImGui::Text("Marker preview");

    ImVec2 window_position = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();

    ImDrawList* imgui_draw_list = ImGui::GetWindowDrawList();
    ImVec2 current_cursor_pos = ImGui::GetCursorPos();
    ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY()};
    imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + 100, local_cursor_pos.y + 100}, ImColor(0, 0, 0, 255), 0, 0);
    ImVec2 center = {local_cursor_pos.x + 100 / 2, local_cursor_pos.y + 100 / 2};

    int marker_style = visuals::aimbot_visual_settings.marker_style;
    int marker_size = visuals::aimbot_visual_settings.marker_size;
    int marker_thickness = visuals::aimbot_visual_settings.marker_thickness;
    ImColor marker_colour = visuals::aimbot_visual_settings.marker_colour;

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
        default:
            break;
    }

    ImGui::EndChild();
    ImGui::EndGroup();
}

void DrawRoutesMenu(void) {
    return;
}

void DrawRadarMenu(void) {
    int right_child_width = 200;

    ImGui::BeginGroup();
    ImGui::BeginChild("left_settings##left_settings", ImVec2(ImGui::GetWindowContentRegionWidth() > right_child_width ? ImGui::GetWindowContentRegionWidth() - right_child_width : ImGui::GetWindowContentRegionWidth(), 0));

    ImGui::Text("Settings");
    ImGui::Separator();
    ImGui::Checkbox("Enabled##radar_enabled", &radar::radar_settings.enabled);
    ImGui::PushItemWidth(100);
    if (ImGui::SliderInt("Poll rate (Hz)##routes", &radar::radar_settings.radar_poll_frequency, 1, 300)) {
        radar::get_radar_data_timer.SetFrequency(radar::radar_settings.radar_poll_frequency);
    }
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    // ImGui::SliderInt("Window size", &visuals::radar_visual_settings.window_size, 0, 1000);
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::ColorEdit4("Radar backgrond Colour", &visuals::radar_visual_settings.window_background_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::PopItemWidth();
    ImGui::Checkbox("Draw axes", &visuals::radar_visual_settings.draw_axes);
    ImGui::PushItemWidth(100);
    ImGui::SliderInt("Axes thickness", &visuals::radar_visual_settings.axes_thickness, 1, 4);
    ImGui::PopItemWidth();
    ImGui::Checkbox("Show friendlies", &radar::radar_settings.show_friendlies);
    ImGui::Checkbox("Show flags", &radar::radar_settings.show_flags);

    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(2 * 0.05, 2 * 0.05, 2 * 0.05, 2 * 0.1));
    ImGui::Separator();
    ImGui::PopStyleColor();

    ImGui::Separator();
    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Zoom", &visuals::radar_visual_settings.zoom, 0, 10);
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::ColorEdit4("Friendly player Colour", &visuals::radar_visual_settings.friendly_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::ColorEdit4("Enemy player Colour", &visuals::radar_visual_settings.enemy_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::ColorEdit4("Friendly flag Colour", &visuals::radar_visual_settings.friendly_flag_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::ColorEdit4("Enemy flag Colour", &visuals::radar_visual_settings.enemy_flag_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::PopItemWidth();

    ImGui::EndChild();
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::SetCursorPosX({ImGui::GetCursorPos().x + 10});
    ImGui::BeginGroup();
    ImGui::BeginChild("right_settings##right_settings", ImVec2(right_child_width, 0));

    ImGui::Text("Marker settings");
    ImGui::Separator();
    ImGui::PushItemWidth(100 + 0 * ImGui::GetWindowWidth() * 0.75);

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
    ImVec2 local_cursor_pos = {window_position.x + ImGui::GetCursorPosX(), window_position.y + ImGui::GetCursorPosY()};
    imgui_draw_list->AddRectFilled(local_cursor_pos, {local_cursor_pos.x + 100, local_cursor_pos.y + 100}, ImColor(0, 0, 0, 255), 0, 0);
    ImVec2 center = {local_cursor_pos.x + 100 / 2, local_cursor_pos.y + 100 / 2};

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

    ImGui::EndChild();
    ImGui::EndGroup();

    return;

    ImGui::BeginGroup();
    // ImGui::BeginChild("left_settings##radar", ImVec2(ImGui::GetWindowContentRegionWidth() > right_child_width ? ImGui::GetWindowContentRegionWidth() - right_child_width : ImGui::GetWindowContentRegionWidth(), 0));
    ImGui::Text("Radar");
    ImGui::Separator();
    ImGui::Checkbox("Radar enabled", &radar::radar_settings.enabled);
    ImGui::SliderFloat("Zoom", &visuals::radar_visual_settings.zoom, 0, 10);

    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(2 * 0.05, 2 * 0.05, 2 * 0.05, 2 * 0.1));
    ImGui::Separator();
    ImGui::PopStyleColor();

    ImGui::ColorEdit4("Friendly Colour", &visuals::radar_visual_settings.friendly_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::ColorEdit4("Enemy Colour", &visuals::radar_visual_settings.enemy_marker_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);

    // ImGui::EndChild();

    ImGui::EndGroup();
}

void DrawESPMenu(void) {
    ImGui::Text("Settings");
    ImGui::Separator();

    ImGui::Checkbox("Enabled", &esp::esp_settings.enabled);
    ImGui::PushItemWidth(100);
    if (ImGui::SliderInt("Poll rate (Hz)", &esp::esp_settings.poll_frequency, 1, 300)) {
        esp::get_esp_data_timer.SetFrequency(esp::esp_settings.poll_frequency);
    }
    ImGui::PopItemWidth();
    // ImGui::SliderFloat("Player width to height ratio", &esp::esp_settings.width_to_height_ratio, 0, 2);

    // ImGui::Checkbox("Disable Z-Buffer")

    ImGui::Checkbox("Show friendlies", &esp::esp_settings.show_friendlies);

    ImGui::Text("Bounding box settings");
    ImGui::Separator();
    ImGui::PushItemWidth(100);
    ImGui::SliderInt("Box thickness", &visuals::esp_visual_settings.bounding_box_settings.box_thickness, 1, 20);
    ImGui::PopItemWidth();

    // ImGui::Separator();

    ImGui::PushItemWidth(100);
    ImGui::ColorEdit4("Friendly Colour##box", &visuals::esp_visual_settings.bounding_box_settings.friendly_player_box_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::ColorEdit4("Enemy Colour##box", &visuals::esp_visual_settings.bounding_box_settings.enemy_player_box_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::PopItemWidth();

    ImGui::Text("Snaplines settings");
    ImGui::Separator();
    ImGui::Checkbox("Show lines", &esp::esp_settings.show_lines);
    ImGui::PushItemWidth(100);
    ImGui::ColorEdit4("Enemy Colour##line", &visuals::esp_visual_settings.line_settings.enemy_player_line_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::SliderInt("Line thickness", &visuals::esp_visual_settings.line_settings.line_thickness, 1, 20);
    ImGui::PopItemWidth();

    if (false) {
        ImGui::Text("Name settings");
        ImGui::Separator();
        ImGui::Checkbox("Show names", &esp::esp_settings.show_names);
        ImGui::PushItemWidth(100);

        ImGui::SliderInt("Name text scale", &visuals::esp_visual_settings.name_settings.scale, 1, 10);
        ImGui::SliderInt("Name height offset", &visuals::esp_visual_settings.name_settings.name_height_offset, -100, 100);
        ImGui::PopItemWidth();
        ImGui::PushItemWidth(100);
        ImGui::ColorEdit4("Friendly Colour##name", &visuals::esp_visual_settings.name_settings.friendly_name_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
        ImGui::ColorEdit4("Enemy Colour##name", &visuals::esp_visual_settings.name_settings.enemy_name_colour.Value.x, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_None | ImGuiColorEditFlags_AlphaBar);
        ImGui::PopItemWidth();
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

void DrawConfigMenu(void) {
    return;
}

void DrawOtherMenu(void) {
    return;
}

void DrawSkinChangerMenu(void) {
    return;
}

void DrawTrainingMenu(void) {
    return;
}

}  // namespace imgui_menu
}  // namespace imgui

namespace ue {
#define PROCESSEVENT_HOOK_FUNCTION(x) void __fastcall x(UObject* uo, void* unused, UFunction* uf, void* p, void* r)

PROCESSEVENT_HOOK_FUNCTION(UEHookMain) {
    DWORD dwWaitResult = WaitForSingleObject(dx9::game_dx_mutex, INFINITE);
    if (ue::frame_is_ready) {
        ue::frame_is_ready = false;
        ATrHUD_eventPostRenderFor_Parms* param = (ATrHUD_eventPostRenderFor_Parms*)p;
        game_data::local_player_controller = (ATrPlayerController*)param->PC;

        keyManager.checkKeyStates(false);
        game_data::GetGameData();
        if (game_data::my_player.is_valid_) {
            aimbot::Tick();
            esp::Tick();
            radar::Tick();
        }
    }
    ReleaseMutex(dx9::game_dx_mutex);
    // LOG("In PostRenderFor");
}

namespace hooks {
void __declspec(naked) __fastcall ProcessEvent(UObject* uo, void* unused, UFunction* uf, void* p, void* r) {
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

void __fastcall ProcessEventHook(UObject* object, void* unused, UFunction* function, void* p, void* r) {
    if (hooks.find(function) != hooks.end()) {
        vector<_ProcessEvent>& hooks_ = hooks[function];
        for (vector<_ProcessEvent>::iterator hook = hooks_.begin(); hook != hooks_.end(); hook++) {
            (*hook)(object, NULL, function, p, r);
        }
    }
    ProcessEvent(object, NULL, function, p, r);
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
    keyManager.addKey(VK_LCONTROL, &aimbot::Reset);

    Hook32::JumpHook processevent_hook(0x00456F90, (DWORD)hooks::ProcessEventHook);
    UFunction* ufunction = (UFunction*)UObject::FindObject<UFunction>((char*)ue::ufunction_to_hook);
    hooks::AddHook(ufunction, &UEHookMain);
}

void DrawImGuiInUE(void) {
    using namespace imgui;

    // game_data::screen_size = {ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y};
    // game_data::screen_center = {game_data::screen_size.X / 2, game_data::screen_size.Y / 2};

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

            vector<FVector2D>& projections = aimbot::projections_of_predictions;

            for (vector<FVector2D>::iterator projected_prediction = projections.begin(); projected_prediction != projections.end(); projected_prediction++) {
                ImVec2 v((*projected_prediction).X, (*projected_prediction).Y);

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
                    default:
                        break;
                }
            }

            vector<pair<FVector2D, ImColor>>& projections_coloured = aimbot::projections_of_predictions_coloured;

            for (vector<pair<FVector2D, ImColor>>::iterator projected_prediction = projections_coloured.begin(); projected_prediction != projections_coloured.end(); projected_prediction++) {
                ImVec2 v((*projected_prediction).first.X, (*projected_prediction).first.Y);

                ImColor marker_colour = projected_prediction->second;

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
                    default:
                        break;
                }
            }

            // visuals::projections_of_predictions.clear();
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
                float box_size_width = box_size_height * esp::esp_settings.width_to_height_ratio;

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
            // if (config::freshly_loaded_config) {
            //    ImGui::SetNextWindowPos(visuals::radar_visual_settings.window_location);
            //} else {
            //    // ImGui::SetNextWindowPos(visuals::radar_visual_settings.window_location, ImGuiCond_FirstUseEver);
            //}

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

            /*
            if (radar::radar_settings.show_flags) {
                radar::RadarLocation radar_location = radar::friendly_flag_location;

                float theta = radar_location.theta;

                float y = radar_location.r * cos(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;
                float x = radar_location.r * sin(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;

                if (!radar_location.right)
                    x = -abs(x);

                // float z = x;
                // x = y;
                // y = z;

                ImVec2 v(center.x + x, center.y - y);

                switch (visuals::radar_visual_settings.marker_style) {
                    case visuals::MarkerStyle::kDot:
                        imgui_draw_list->AddCircleFilled(v, marker_size, friendly_flag_marker_colour);
                        break;
                    case visuals::MarkerStyle::kCircle:
                        imgui_draw_list->AddCircle(v, marker_size, friendly_flag_marker_colour, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kFilledSquare:
                        imgui_draw_list->AddRectFilled({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, friendly_flag_marker_colour, 0);
                        break;
                    case visuals::MarkerStyle::kSquare:
                        imgui_draw_list->AddRect({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, friendly_flag_marker_colour, 0, 0, marker_thickness);
                        break;
                    default:
                        break;
                }
            }


            if (radar::radar_settings.show_flags) {
                radar::RadarLocation radar_location = radar::enemy_flag_location;

                float theta = radar_location.theta;

                float y = radar_location.r * cos(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;
                float x = radar_location.r * sin(theta) * visuals::radar_visual_settings.zoom_ * visuals::radar_visual_settings.zoom;

                if (!radar_location.right)
                    x = -abs(x);

                // float z = x;
                // x = y;
                // y = z;

                ImVec2 v(center.x + x, center.y - y);

                switch (visuals::radar_visual_settings.marker_style) {
                    case visuals::MarkerStyle::kDot:
                        imgui_draw_list->AddCircleFilled(v, marker_size, enemy_flag_marker_colour);
                        break;
                    case visuals::MarkerStyle::kCircle:
                        imgui_draw_list->AddCircle(v, marker_size, enemy_flag_marker_colour, 0, marker_thickness);
                        break;
                    case visuals::MarkerStyle::kFilledSquare:
                        imgui_draw_list->AddRectFilled({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, enemy_flag_marker_colour, 0);
                        break;
                    case visuals::MarkerStyle::kSquare:
                        imgui_draw_list->AddRect({v.x - marker_size, v.y - marker_size}, {v.x + marker_size, v.y + marker_size}, enemy_flag_marker_colour, 0, 0, marker_thickness);
                        break;
                    default:
                        break;
                }
            }
            */

            if (dx9::imgui_show_menu) {
                ImGui::PopStyleColor();
            }
            ImGui::End();
            // ImGui::PopStyleColor();
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

    if (dx9::imgui_show_menu) {
        static bool unused_boolean = true;

        ImGuiStyle& style = ImGui::GetStyle();
        ImVec4* colors = style.Colors;

        ImGui::SetNextWindowPos({game_data::screen_center.X / 2, game_data::screen_center.Y / 2}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({600, 500}, ImGuiCond_FirstUseEver);

        ImGui::Begin("descension v0.4 | MACE 1.9.1.12285", NULL, ImGuiWindowFlags_AlwaysAutoResize & 0);

        ImVec2 window_position = ImGui::GetWindowPos();
        ImVec2 window_size = ImGui::GetWindowSize();
        ImVec2 center(window_position.x + window_size.x / 2, window_position.y + window_size.y / 2);

        ImGui::PushStyleColor(ImGuiCol_ChildBg, colors[ImGuiCol_WindowBg]);
        ImGui::BeginChild("left_child##left_menu", {150, 0}, false);
        ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0, 0));

        for (int i = 0; i < imgui_menu::buttons_num; i++) {
            ImVec2 size = ImVec2(ImGui::GetWindowWidth() * .75, 0);
            bool b_selected = i == imgui_menu::selected_index;
            if (b_selected) {
                size.x = ImGui::GetWindowWidth() * 0.9;
                // ImGui::PushFont(font_selected_item);
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
                ImGui::PopStyleVar();
            }
        }

        ImGui::PopStyleVar();
        ImGui::EndChild();
        ImGui::SameLine();

        // ImGui::NextColumn();

        ImGui::BeginChild("right_child##right_menu");

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
            case imgui_menu::LeftMenuButtons::kRoutes:
                imgui_menu::DrawRoutesMenu();
                break;
            case imgui_menu::LeftMenuButtons::kOptions:
                // imgui_menu::DrawOptionsMenu();
                break;
            case imgui_menu::LeftMenuButtons::kCrosshair:
                imgui_menu::DrawCrosshairMenu();
                break;
            case imgui_menu::LeftMenuButtons::kConfigs:
                imgui_menu::DrawConfigMenu();
                break;
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
        }

        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::End();
    }
}
}  // namespace ue