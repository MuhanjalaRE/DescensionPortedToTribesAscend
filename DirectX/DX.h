#pragma once

#include <Windows.h>

namespace dx9 {
extern HANDLE game_dx_mutex;
extern bool imgui_show_menu;

void Hook(void);
}  // namespace dx9