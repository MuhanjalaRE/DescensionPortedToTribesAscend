#include "Hook.h"

#include <iostream>

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

bool MidFunctionHook::Hook(DWORD source, DWORD destination, unsigned int hook_length) {
    if (!hooked_) {
        if (hook_length < minimum_number_of_bytes_to_overwrite_) {
            return false;
        }

        static BYTE asm_jump = 0xE9;
        static BYTE asm_nop = 0x90;
        static BYTE asm_push = 0x68;
        static BYTE asm_pushad = 0x60;
        static BYTE asm_popad = 0x61;
        static BYTE asm_ret = 0xC3;

        // Save the original bytes
        original_bytes_ = new BYTE[hook_length];
        memcpy(original_bytes_, (void*)source, hook_length);

        // Get the address of the next instruction after where the hook is being placed
        DWORD next_instruction_address = source + hook_length;

        /*
        __asm {
            push next_instruction_address // (5 bytes) When the buffer returns, it will jump to this address
            pushad //(1 byte) Save all the GPR
            push eip + 4 + 5 //(5 bytes) After the jmp on the next line, we want to return to the instruction after the jump
            jmp destination - eip  - 5 // (5 bytes) ( Jump to the destination hook function
            popad // (1 byte) Once the destination hook function returns, it will come to this instruction (the ret will pop the pushed address two lines above). Restore all GPR
            // (hook_size bytes) Copy all the original instructions here
            ret // (1 byte) Return to the address at next_instruction_address and continue functioning

        }
        */

        // Create the buffer
        unsigned int buffer_size = 5 + 1 + 5 + 5 + 1 + hook_length + 1;
        executeable_buffer_ = VirtualAlloc(NULL, hook_length, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        BYTE* executeable_buffer = (BYTE*)executeable_buffer_;

        int index = 0;

        executeable_buffer[index] = asm_push;
        index += sizeof(BYTE);

        *((DWORD*)(executeable_buffer + index)) = next_instruction_address;
        index += sizeof(DWORD);

        executeable_buffer[index] = asm_pushad;
        index += sizeof(BYTE);

        executeable_buffer[index] = asm_push;
        index += sizeof(BYTE);

        *((DWORD*)(executeable_buffer + index)) = (DWORD)executeable_buffer_ + index + 4 + 5;
        index += sizeof(long);

        executeable_buffer[index] = asm_jump;
        *((long*)(executeable_buffer + index + 1)) = destination - ((DWORD)executeable_buffer + index) - 5;

        index += sizeof(BYTE);
        index += sizeof(long);

        executeable_buffer[index] = asm_popad;
        index += sizeof(BYTE);

        memcpy((void*)(executeable_buffer + index), original_bytes_, hook_length);
        index += hook_length;

        executeable_buffer[index] = asm_ret;
        index += sizeof(BYTE);

        DWORD protection;
        VirtualProtect(executeable_buffer_, buffer_size, PAGE_EXECUTE_READ, &protection);

        VirtualProtect((void*)source, hook_length, PAGE_EXECUTE_READWRITE, &protection);

        *((BYTE*)source) = asm_jump;
        *((long*)(source + 1)) = (DWORD)executeable_buffer_ - source - 5;  // Jump to the executeable buffer

        for (int i = 0; i < hook_length - minimum_number_of_bytes_to_overwrite_; i++) {
            *((BYTE*)(source + minimum_number_of_bytes_to_overwrite_ + i)) = asm_nop;
        }

        VirtualProtect((void*)source, hook_length, protection, &protection);

        hook_address_ = source;
        hooked_ = true;
        return true;
    } else {
        return false;
    }
}

MidFunctionHook::MidFunctionHook(DWORD source, DWORD destination, unsigned int hook_length) {
    bool result = Hook(source, destination, hook_length);
}

}  // namespace Hook32


namespace Hook64 {
bool Hook64::IsHooked(void) {
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

bool MidFunctionHook::Hook(DWORD source, DWORD destination, unsigned int hook_length) {
    if (!hooked_) {
        if (hook_length < minimum_number_of_bytes_to_overwrite_) {
            return false;
        }

        static BYTE asm_jump = 0xE9;
        static BYTE asm_nop = 0x90;
        static BYTE asm_push = 0x68;
        static BYTE asm_pushad = 0x60;
        static BYTE asm_popad = 0x61;
        static BYTE asm_ret = 0xC3;

        // Save the original bytes
        original_bytes_ = new BYTE[hook_length];
        memcpy(original_bytes_, (void*)source, hook_length);

        // Get the address of the next instruction after where the hook is being placed
        DWORD next_instruction_address = source + hook_length;

        /*
        __asm {
            movabs rax, next_instruction_address (10 bytes) When the buffer returns, it will jump to this address
            push rax (2 bytes)

            pushad //(1 byte) Save all the GPR

            movabs rax, after_jump_address (*) // (10 bytes) After the jmp on the next line, we want to return to the instruction after the jump
            push rax (2 bytes)
            
            movabs rax, destination (10 bytes (2 opcode, 8 dest)) // Move hook destination in to rax
            jmp rax (2 bytes) // Jmp to rax
            
            (*) popad // (1 byte) Once the destination hook function returns, it will come to this instruction (the ret will pop the pushed address two lines above). Restore all GPR
            
            // (hook_size bytes) Copy all the original instructions here
            
            ret // (1 byte) Return to the address at next_instruction_address and continue functioning

        }
        */

        // Create the buffer
        unsigned int buffer_size = 5 + 1 + 5 + 5 + 1 + hook_length + 1;
        executeable_buffer_ = VirtualAlloc(NULL, hook_length, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        BYTE* executeable_buffer = (BYTE*)executeable_buffer_;

        int index = 0;

        executeable_buffer[index] = asm_push;
        index += sizeof(BYTE);

        *((DWORD*)(executeable_buffer + index)) = next_instruction_address;
        index += sizeof(DWORD);

        executeable_buffer[index] = asm_pushad;
        index += sizeof(BYTE);

        executeable_buffer[index] = asm_push;
        index += sizeof(BYTE);

        *((DWORD*)(executeable_buffer + index)) = (DWORD)executeable_buffer_ + index + 4 + 5;
        index += sizeof(long);

        executeable_buffer[index] = asm_jump;
        *((long*)(executeable_buffer + index + 1)) = destination - ((DWORD)executeable_buffer + index) - 5;

        index += sizeof(BYTE);
        index += sizeof(long);

        executeable_buffer[index] = asm_popad;
        index += sizeof(BYTE);

        memcpy((void*)(executeable_buffer + index), original_bytes_, hook_length);
        index += hook_length;

        executeable_buffer[index] = asm_ret;
        index += sizeof(BYTE);

        DWORD protection;
        VirtualProtect(executeable_buffer_, buffer_size, PAGE_EXECUTE_READ, &protection);

        VirtualProtect((void*)source, hook_length, PAGE_EXECUTE_READWRITE, &protection);

        *((BYTE*)source) = asm_jump;
        *((long*)(source + 1)) = (DWORD)executeable_buffer_ - source - 5;  // Jump to the executeable buffer

        for (int i = 0; i < hook_length - minimum_number_of_bytes_to_overwrite_; i++) {
            *((BYTE*)(source + minimum_number_of_bytes_to_overwrite_ + i)) = asm_nop;
        }

        VirtualProtect((void*)source, hook_length, protection, &protection);

        hook_address_ = source;
        hooked_ = true;
        return true;
    } else {
        return false;
    }
}

MidFunctionHook::MidFunctionHook(DWORD source, DWORD destination, unsigned int hook_length) {
    bool result = Hook(source, destination, hook_length);
}

}  // namespace Hook32