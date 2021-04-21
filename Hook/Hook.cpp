#include "Hook.h"

namespace Hook32 {
bool Hook32::IsHooked(void) {
    return this->hooked_;
}

bool JumpHook::Hook(DWORD source, DWORD destination) {
    if (!hooked_) {
        memcpy((void*)original_bytes_, (void*)source, number_of_bytes_to_overwrite_);

        static BYTE asm_jump = 0xE9;
        long offset = destination - source - number_of_bytes_to_overwrite_;

        DWORD protection;
        VirtualProtect((void*)source, number_of_bytes_to_overwrite_, PAGE_EXECUTE_READWRITE, &protection);

        *((BYTE*)source) = asm_jump;
        *((long*)(source + 1)) = offset;
        VirtualProtect((void*)source, number_of_bytes_to_overwrite_, protection, &protection);

        hook_address_ = source;
        hooked_ = true;
        return true;
    } else {
        return false;
    }
}

bool JumpHook::UnHook(void) {
    if (hooked_) {
        DWORD protection;
        VirtualProtect((void*)hook_address_, number_of_bytes_to_overwrite_, PAGE_EXECUTE_READWRITE, &protection);

        memcpy((void*)hook_address_, (void*)original_bytes_, number_of_bytes_to_overwrite_);

        VirtualProtect((void*)hook_address_, number_of_bytes_to_overwrite_, protection, &protection);

        hooked_ = false;
        return true;
    } else {
        return false;
    }
}

JumpHook::JumpHook(DWORD source, DWORD destination) {
    bool result = Hook(source, destination);
}

DWORD JumpHook::GetHookAddress(void) {
    if (hooked_) {
        return hook_address_;
    } else {
        return 0;
    }
}

bool VMTHook::Hook(void* object, DWORD function_index, void* destination_function) {
    if (!hooked_) {
        DWORD object_address = (DWORD)object;
        DWORD vmt_address = *((DWORD*)object_address);
        DWORD function_address = vmt_address + sizeof(DWORD) * function_index;
        DWORD function_address_value = *((DWORD*)function_address);

        DWORD protection;
        VirtualProtect((void*)function_address, sizeof(DWORD), PAGE_EXECUTE_READWRITE, &protection);

        original_address_ = *((DWORD*)function_address);
        *((DWORD*)function_address) = (DWORD)destination_function;

        VirtualProtect((void*)function_address, sizeof(DWORD), protection, &protection);

        hook_address_ = function_address_value;
        hooked_ = true;
        return true;
    } else {
        return false;
    }
}

bool VMTHook::UnHook(void) {
    if (hooked_) {
        DWORD protection;
        VirtualProtect((void*)hook_address_, sizeof(DWORD), PAGE_EXECUTE_READWRITE, &protection);

        *((DWORD*)hook_address_) = original_address_;

        VirtualProtect((void*)hook_address_, sizeof(DWORD), protection, &protection);
        return true;
    } else {
        return false;
    }
}

VMTHook::VMTHook(void* object, DWORD function_index, void* destination_function) {
    bool result = Hook(object, function_index, destination_function);
}

DWORD VMTHook::GetFunctionFirstInstructionAddress(void* object, DWORD function_index) {
    DWORD object_address = (DWORD)object;
    DWORD vmt_address = *((DWORD*)object_address);
    DWORD function_address = vmt_address + sizeof(DWORD) * function_index;
    DWORD first_instruction_address = *((DWORD*)function_address);
    return first_instruction_address;
}

DWORD VMTHook::GetOriginalFunction(void) {
    if (hooked_) {
        return hook_address_;
    } else {
        return NULL;
    }
}

}  // namespace Hook32