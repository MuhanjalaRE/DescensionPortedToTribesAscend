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

namespace ue {
#define PROCESSEVENT_HOOK_FUNCTION(x) void __fastcall x(UObject* uo, void* unused, UFunction* uf, void* p, void* r)

PROCESSEVENT_HOOK_FUNCTION(UEHookMain) {
    LOG("In PostRenderFor");
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
    Hook32::JumpHook processevent_hook(0x00456F90, (DWORD)hooks::ProcessEventHook);
    UFunction* ufunction = (UFunction*)UObject::FindObject<UFunction>((char*)ue::ufunction_to_hook);
    hooks::AddHook(ufunction, &UEHookMain);
}

void DrawImGuiInUE(void) {}
}  // namespace ue