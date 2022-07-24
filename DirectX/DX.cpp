#pragma comment(lib, "d3d9.lib")
#pragma comment(lib, "d3dx9.lib")

#include <d3d9.h>
#include <d3dx9.h>

#include <iostream>
#include <string>

#include "DX.h"
#include "Hook.h"
#include "log.h"
#include "ue.h"

#include "imgui.h"
#include "imgui_impl_dx9.h"
#include "imgui_impl_win32.h"
#include "imgui_internal.h"

#include "detours.h"

#define USE_MID_FUNCTION_PRESENT_HOOK
#define HWND_WINDOW_NAME "Tribes: Ascend (32-bit, DX9)"

extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

using namespace std;
using namespace Hook32;

extern bool ue::frame_is_ready = false;

namespace imgui {
void Setup(void);

void Setup(void) {
    ImGuiStyle& style = ImGui::GetStyle();
    style.TabRounding = 0.0f;
    style.FrameBorderSize = 1.0f;
    style.ScrollbarRounding = 0.0f;
    style.ScrollbarSize = 10.0f;
    style.WindowTitleAlign = ImVec2(0.5f, 0.5f);
    style.CellPadding = ImVec2(6, 3);
    style.ItemSpacing.y = 8;
    style.FrameRounding = 2;
    style.WindowRounding = 8;
    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_Text] = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.08f, 0.12f, 0.92f);  // ImVec4(0.12f, 0.12f, 0.12f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.04f, 0.04f, 0.04f, 0.50f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.12f, 0.12f, 0.12f, 0.94f);
    colors[ImGuiCol_Border] = ImVec4(0.25f, 0.25f, 0.27f, 0.50f);
    colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.20f, 0.22f, 0.50f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.25f, 0.25f, 0.27f, 0.75f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.30f, 0.33f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.04f, 0.04f, 0.04f, 0.92f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.04f, 0.04f, 0.04f, 0.92f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.04f, 0.04f, 0.04f, 0.75f);
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.18f, 0.18f, 0.19f, 1.00f);
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.24f, 0.24f, 0.26f, 0.75f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.41f, 0.41f, 0.41f, 0.75f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.62f, 0.62f, 0.62f, 0.75f);
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.94f, 0.92f, 0.94f, 0.75f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.41f, 0.41f, 0.41f, 0.75f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.62f, 0.62f, 0.62f, 0.75f);
    colors[ImGuiCol_Button] = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.25f, 0.25f, 0.27f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.18f, 0.18f, 0.19f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.25f, 0.25f, 0.27f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.25f, 0.25f, 0.27f, 1.00f);
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.62f, 0.62f, 0.62f, 1.00f);
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.30f, 0.30f, 0.33f, 0.75f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.41f, 0.41f, 0.41f, 0.75f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.62f, 0.62f, 0.62f, 0.75f);
    colors[ImGuiCol_Tab] = ImVec4(0.21f, 0.21f, 0.22f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.37f, 0.37f, 0.39f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.30f, 0.30f, 0.33f, 1.00f);
    colors[ImGuiCol_TabUnfocused] = ImVec4(0.12f, 0.12f, 0.12f, 0.97f);
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.18f, 0.18f, 0.19f, 1.00f);
    colors[ImGuiCol_PlotLines] = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    colors[ImGuiCol_TextSelectedBg] = ImVec4(0.26f, 0.59f, 0.98f, 0.50f);
    colors[ImGuiCol_DragDropTarget] = ImVec4(1.00f, 1.00f, 0.00f, 0.90f);
    colors[ImGuiCol_NavHighlight] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
    colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.35f);
    style.WindowMenuButtonPosition = ImGuiDir_Right;

#ifdef IMGUI_HAS_DOCK
    colors[ImGuiCol_DockingPreview] = ImVec4(0.26f, 0.59f, 0.98f, 0.50f);
    colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
#endif
}

}  // namespace imgui

namespace dx9 {

typedef HRESULT(__stdcall* BeginScene)(LPDIRECT3DDEVICE9);
typedef HRESULT(__stdcall* EndScene)(LPDIRECT3DDEVICE9);
typedef HRESULT(__stdcall* Reset)(LPDIRECT3DDEVICE9, D3DPRESENT_PARAMETERS*);
typedef HRESULT(__stdcall* DrawIndexedPrimitive)(LPDIRECT3DDEVICE9, D3DPRIMITIVETYPE, INT, UINT, UINT, UINT, UINT);
typedef HRESULT(__stdcall* DrawPrimitive)(LPDIRECT3DDEVICE9, D3DPRIMITIVETYPE, UINT, UINT);
typedef HRESULT(__stdcall* DrawIndexedPrimitiveUP)(LPDIRECT3DDEVICE9, D3DPRIMITIVETYPE, UINT, UINT, UINT, CONST void*, D3DFORMAT, CONST void*, UINT);

typedef HRESULT(__stdcall* SetTexture)(LPDIRECT3DDEVICE9, DWORD, IDirect3DBaseTexture9*);

typedef HRESULT(__stdcall* Present)(LPDIRECT3DDEVICE9, const RECT*, const RECT*, HWND, const RGNDATA*);

HRESULT __stdcall BeginSceneHook(LPDIRECT3DDEVICE9 device);
HRESULT __stdcall EndSceneGetDeviceHook(LPDIRECT3DDEVICE9 device);

HRESULT __stdcall EndSceneHook(LPDIRECT3DDEVICE9 device);
HRESULT __stdcall ResetHook(LPDIRECT3DDEVICE9 device, D3DPRESENT_PARAMETERS* pPresentationParameters);

HRESULT __stdcall DrawIndexedPrimitiveHook(LPDIRECT3DDEVICE9 device, D3DPRIMITIVETYPE primType, INT BaseVertexIndex, UINT MinVertexIndex, UINT NumVertices, UINT startIndex, UINT primCount);

HRESULT __stdcall DrawPrimitiveHook(LPDIRECT3DDEVICE9 device, D3DPRIMITIVETYPE PrimitiveType, UINT StartVertex, UINT PrimitiveCount);
HRESULT __stdcall DrawIndexedPrimitiveUPHook(LPDIRECT3DDEVICE9 device, D3DPRIMITIVETYPE PrimitiveType, UINT MinVertexIndex, UINT NumVertices, UINT PrimitiveCount, CONST void* pIndexData, D3DFORMAT IndexDataFormat, CONST void* pVertexStreamZeroData, UINT VertexStreamZeroStride);

HRESULT __stdcall SetTextureHook(LPDIRECT3DDEVICE9 device, DWORD Stage, IDirect3DBaseTexture9* pTexture);

HRESULT __stdcall PresentHook(LPDIRECT3DDEVICE9 device, const RECT* pSourceRect, const RECT* pDestRect, HWND hDestWindowOverride, const RGNDATA* pDirtyRegion);

bool CreateDevice(void);
// bool LoadTextureFromFile(const char* filename, PDIRECT3DTEXTURE9* out_texture, int* out_width, int* out_height);
bool LoadTextureFromFile(const char* filename, class Image** out_image);

LRESULT WINAPI CustomWindowProcCallback(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static LPDIRECT3DDEVICE9 game_device = NULL;
static WNDPROC original_windowproc_callback = NULL;
static HWND game_hwnd = NULL;

VMTHook* begin_scene_vmthook;
VMTHook* end_scene_vmthook;
VMTHook* reset_vmthook;
VMTHook* draw_indexed_primitive_vmthook;
VMTHook* draw_primitive_vmthook;
VMTHook* draw_indexed_primitive_up_vmthook;
VMTHook* set_texture_vmthook;
VMTHook* present_vmthook;
JumpHook* end_scene_get_device_jumphook;

extern HANDLE game_dx_mutex = CreateMutex(NULL, false, NULL);
bool imgui_show_menu = false;
bool imgui_is_ready = false;

// bool resolution_init = false;
// ImVec2 resolution;

DWORD present_mid_function_hook_end = NULL;
void PresentMidFunctionHookFunction(void);
void PresentMidFunctionHook(void);
void DoImGuiDrawing(void);

}  // namespace dx9

namespace dx9 {
struct Image {
    PDIRECT3DTEXTURE9 texture_;
    int width_;
    int height_;
    Image(PDIRECT3DTEXTURE9 texture, int width, int height) {
        texture_ = texture;
        width_ = width;
        height_ = height;
    }
}* crosshair = NULL;
}  // namespace dx9

namespace dx9 {
void Hook(void) {
#ifdef LOG
    AllocConsole();
    freopen("CONOUT$", "w", stdout);
#endif

    if (CreateDevice()) {
        LOG("Successfully created a LPDIRECT3DDEVICE9 object.");
    } else {
        LOG("Failed to create LPDIRECT3DDEVICE9 object.");
    }
}

bool CreateDevice(void) {
    LPDIRECT3D9 direct3d9_object = Direct3DCreate9(D3D_SDK_VERSION);
    if (!direct3d9_object) {
        LOG("Attempted to create LPDIRECT3D9 object but Direct3DCreate9 function call failed.");
    }

    HWND hwnd = FindWindowA(NULL, HWND_WINDOW_NAME);
    if (!hwnd) {
        LOG("Failed to find hwnd after calling FindWindowA function.");
    }

    game_hwnd = hwnd;

    original_windowproc_callback = (WNDPROC)SetWindowLongPtr(hwnd, GWL_WNDPROC, (LONG_PTR)CustomWindowProcCallback);

    D3DPRESENT_PARAMETERS d3dpresent_parameters;
    ZeroMemory(&d3dpresent_parameters, sizeof(d3dpresent_parameters));
    d3dpresent_parameters.Windowed = TRUE;
    d3dpresent_parameters.SwapEffect = D3DSWAPEFFECT_DISCARD;
    d3dpresent_parameters.hDeviceWindow = hwnd;
    LPDIRECT3DDEVICE9 new_device;
    HRESULT result = direct3d9_object->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpresent_parameters, &new_device);

    if (FAILED(result)) {
        LOG("Error creating device after calling CreateDevice function.");
        return false;
    }

    DWORD end_scene_first_instruction_address = VMTHook::GetFunctionFirstInstructionAddress(new_device, 42);

    end_scene_get_device_jumphook = new JumpHook(end_scene_first_instruction_address, (DWORD)EndSceneGetDeviceHook);

    direct3d9_object->Release();
    new_device->Release();

    LOG("Sucessfully created a new DX9 device.");

    return true;
}

bool LoadTextureFromFile(const char* filename, Image** out_image) {
    // Load texture from disk
    PDIRECT3DTEXTURE9 texture;
    HRESULT hr = D3DXCreateTextureFromFileA(game_device, filename, &texture);
    if (hr != S_OK)
        return false;

    // Retrieve description of the texture surface so we can access its size
    D3DSURFACE_DESC my_image_desc;
    texture->GetLevelDesc(0, &my_image_desc);

    *out_image = new Image(texture, (int)my_image_desc.Width, (int)my_image_desc.Height);
    return true;
}

LRESULT WINAPI CustomWindowProcCallback(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (msg == WM_KEYDOWN) {
        if (wParam == VK_INSERT) {
            imgui_show_menu = !imgui_show_menu;
        } else if (wParam == 17) {  // LCONTROL
            aimbot::Reset();
        } else if (wParam == 16) {  // LSHIFT
            aimbot::aimbot_enabled = true;
        }
    } else if (msg == WM_KEYUP) {
        if (wParam == 16) {  // LSHIFT
            aimbot::aimbot_enabled = false;
        }
    }

    ImGuiIO& io = ::ImGui::GetIO();
    if (imgui_show_menu) {
        io.MouseDrawCursor = true;
        ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam);
        return true;  // Comment this line to allow keyboard+mouse input done in the menu to pass through into the actual game
    } else {
        io.MouseDrawCursor = false;
    }

    return CallWindowProc(original_windowproc_callback, hWnd, msg, wParam, lParam);
}

void DoImGuiDrawing(void) {
    static ImGuiIO* io = nullptr;
    if (!io) {
        io = &ImGui::GetIO();
    }

    static ImFont* font = NULL;
    if (!font && io) {
        ImFontConfig config_;
        config_.SizePixels = (int)(((ImFont*)io->Fonts->AddFontDefault())->FontSize * 2);

        font = io->Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\Arial.ttf", 16, NULL, io->Fonts->GetGlyphRangesDefault());
        ImFontConfig config;
        config.MergeMode = true;
        config.GlyphMinAdvanceX = 0.0f;  // Use if you want to make the icon monospaced
        static const ImWchar icon_ranges[] = {0x25A0, 0x25FF, 0};
        io->Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\L_10646.ttf", 32, &config, icon_ranges);
        io->Fonts->Build();
    }

    DWORD dwWaitResult = WaitForSingleObject(game_dx_mutex, INFINITE);

    ImGui_ImplDX9_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    if (font) {
        ImGui::PushFont(font);
    }

    ue::DrawImGuiInUE();

    if (font) {
        ImGui::PopFont();
    }

    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplDX9_RenderDrawData(ImGui::GetDrawData());

    ue::frame_is_ready = true;
    ReleaseMutex(game_dx_mutex);
}

HRESULT __stdcall EndSceneGetDeviceHook(LPDIRECT3DDEVICE9 device) {
    LOG("In EndSceneGetDeviceHook function.");

    DWORD original_function_address = end_scene_get_device_jumphook->GetHookAddress();

    end_scene_get_device_jumphook->UnHook();

    game_device = device;

    begin_scene_vmthook = new VMTHook(game_device, 41, BeginSceneHook);
    end_scene_vmthook = new VMTHook(game_device, 42, EndSceneHook);
    reset_vmthook = new VMTHook(game_device, 16, ResetHook);
    draw_indexed_primitive_vmthook = new VMTHook(game_device, 82, DrawIndexedPrimitiveHook);
    draw_primitive_vmthook = new VMTHook(game_device, 81, DrawPrimitiveHook);
    draw_indexed_primitive_up_vmthook = new VMTHook(game_device, 84, DrawIndexedPrimitiveUPHook);
    set_texture_vmthook = new VMTHook(game_device, 65, SetTextureHook);

#ifdef USE_MID_FUNCTION_PRESENT_HOOK_
    // PrintMemory((void*)VMTHook::GetFunctionFirstInstructionAddress(game_device, 17), 300);
    DWORD present_mid_function_hook_address = VMTHook::GetFunctionFirstInstructionAddress(game_device, 17) + 0x26;
    unsigned int hook_size = 0x2d - 0x26;
    present_mid_function_hook_end = present_mid_function_hook_address + hook_size;
    JumpHook present_mid_function_jumphook(present_mid_function_hook_address, (DWORD)PresentMidFunctionHook);

    if (hook_size > JumpHook::number_of_bytes_to_overwrite_) {
        // NOP the extra assembly instructions
        static const BYTE asm_nop = 0x90;
        DWORD protection;
        VirtualProtect((void*)(present_mid_function_hook_address + JumpHook::number_of_bytes_to_overwrite_), hook_size - JumpHook::number_of_bytes_to_overwrite_, PAGE_EXECUTE_READWRITE, &protection);
        for (int i = 0; i < hook_size - JumpHook::number_of_bytes_to_overwrite_; i++) {
            *((BYTE*)(present_mid_function_hook_address + JumpHook::number_of_bytes_to_overwrite_ + i)) = asm_nop;
        }
        VirtualProtect((void*)(present_mid_function_hook_address + JumpHook::number_of_bytes_to_overwrite_), hook_size - JumpHook::number_of_bytes_to_overwrite_, protection, &protection);
    }
    // PrintMemory((void*)VMTHook::GetFunctionFirstInstructionAddress(game_device, 17), 300);
#endif

#ifdef USE_MID_FUNCTION_PRESENT_HOOK
    MidFunctionHook(VMTHook::GetFunctionFirstInstructionAddress(game_device, 17) + 0x26, (DWORD)PresentMidFunctionHookFunction, 0x2d - 0x26);
#else
    present_vmthook = new VMTHook(game_device, 17, PresentHook);
#endif

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = NULL;
    (void)io;

    ImGui_ImplWin32_Init(game_hwnd);
    ImGui_ImplDX9_Init(game_device);

    HRESULT res = ((EndScene)original_function_address)(device);

    delete end_scene_get_device_jumphook;

    /*
    bool load_texture_result = LoadTextureFromFile("crosshair.png", &crosshair);
    if (load_texture_result) {
        LOG("Successfully loaded texture.");
    } else {
        LOG("Failed to load texture.");
    }
    */

    imgui::Setup();

    LOG("Hooking Unreal Engine.");
    ue::HookUnrealEngine();

    return res;
}

HRESULT __stdcall BeginSceneHook(LPDIRECT3DDEVICE9 device) {
    // LOG("In BeginSceneHook function.");

    HRESULT result = ((BeginScene)begin_scene_vmthook->GetOriginalFunction())(device);
    return result;
}

HRESULT __stdcall EndSceneHook(LPDIRECT3DDEVICE9 device) {
    // LOG("In EndSceneHook function.");

#ifndef USE_MID_FUNCTION_PRESENT_HOOK
    DoImGuiDrawing();
#endif

    HRESULT result = ((EndScene)end_scene_vmthook->GetOriginalFunction())(device);
    return result;
}

HRESULT __stdcall ResetHook(LPDIRECT3DDEVICE9 device, D3DPRESENT_PARAMETERS* pPresentationParameters) {
    ImGui_ImplDX9_InvalidateDeviceObjects();
    HRESULT result = ((Reset)reset_vmthook->GetOriginalFunction())(device, pPresentationParameters);
    ImGui_ImplDX9_CreateDeviceObjects();
    return result;
}

HRESULT __stdcall DrawIndexedPrimitiveHook(LPDIRECT3DDEVICE9 device, D3DPRIMITIVETYPE primType, INT BaseVertexIndex, UINT MinVertexIndex, UINT NumVertices, UINT startIndex, UINT primCount) {
    string primCount_string = std::to_string(primCount);

    HRESULT result = ((DrawIndexedPrimitive)draw_indexed_primitive_vmthook->GetOriginalFunction())(device, primType, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);
    return result;
}

HRESULT __stdcall DrawPrimitiveHook(LPDIRECT3DDEVICE9 device, D3DPRIMITIVETYPE PrimitiveType, UINT StartVertex, UINT PrimitiveCount) {
    HRESULT result = ((DrawPrimitive)draw_primitive_vmthook->GetOriginalFunction())(device, PrimitiveType, StartVertex, PrimitiveCount);
    return result;
}

HRESULT __stdcall DrawIndexedPrimitiveUPHook(LPDIRECT3DDEVICE9 device, D3DPRIMITIVETYPE PrimitiveType, UINT MinVertexIndex, UINT NumVertices, UINT PrimitiveCount, CONST void* pIndexData, D3DFORMAT IndexDataFormat, CONST void* pVertexStreamZeroData, UINT VertexStreamZeroStride) {
    HRESULT result = ((DrawIndexedPrimitiveUP)draw_indexed_primitive_up_vmthook->GetOriginalFunction())(device, PrimitiveType, MinVertexIndex, NumVertices, PrimitiveCount, pIndexData, IndexDataFormat, pVertexStreamZeroData, VertexStreamZeroStride);
    return result;
}

HRESULT __stdcall SetTextureHook(LPDIRECT3DDEVICE9 device, DWORD Stage, IDirect3DBaseTexture9* pTexture) {
    HRESULT result = ((SetTexture)(set_texture_vmthook->GetOriginalFunction()))(device, Stage, pTexture);
    return result;
}

HRESULT __stdcall PresentHook(LPDIRECT3DDEVICE9 device, const RECT* pSourceRect, const RECT* pDestRect, HWND hDestWindowOverride, const RGNDATA* pDirtyRegion) {
    HRESULT result = ((Present)(present_vmthook->GetOriginalFunction()))(device, pSourceRect, pDestRect, hDestWindowOverride, pDirtyRegion);
    return result;
}

void PresentMidFunctionHookFunction(void) {
    DoImGuiDrawing();
}

__declspec(naked) void PresentMidFunctionHook(void) {
    // call something

    __asm {
        pushad
    }

    PresentMidFunctionHookFunction();

    __asm {
        popad
            // do the instructions overwritten
        test DWORD PTR [esi+0x30],0x2
        mov eax, present_mid_function_hook_end
        jmp eax
    }
}

}  // namespace dx9