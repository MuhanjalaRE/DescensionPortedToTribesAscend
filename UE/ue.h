#pragma once
namespace ue {
extern bool frame_is_ready;
extern const char* ufunction_to_hook;

void HookUnrealEngine(void);
void DrawImGuiInUE(void);
}

namespace aimbot {
extern bool aimbot_enabled;
void Reset(void);
}