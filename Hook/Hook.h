#pragma once
#include <Windows.h>

namespace Hook32 {
class Hook32 {
   protected:
    bool hooked_ = false;

   public:
    bool IsHooked(void);
};

class JumpHook : public Hook32 {
   private:
    static const BYTE number_of_bytes_to_overwrite_ = 5;
    BYTE original_bytes_[number_of_bytes_to_overwrite_];
    DWORD hook_address_;

   public:
    bool Hook(DWORD source, DWORD destination);
    bool UnHook(void);
    JumpHook(DWORD source, DWORD destination);
    DWORD GetHookAddress(void);
};

class VMTHook : public Hook32 {
   private:
    DWORD hook_address_;
    DWORD original_address_;

   public:
    bool Hook(void* object, DWORD function_index, void* destination_function);
    bool UnHook(void);
    VMTHook(void* object, DWORD function_index, void* destination_function);
    static DWORD GetFunctionFirstInstructionAddress(void* object, DWORD function_index);
    DWORD GetOriginalFunction(void);
};

}  // namespace Hook32