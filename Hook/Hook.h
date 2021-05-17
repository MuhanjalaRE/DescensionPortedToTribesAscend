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
   public:
    static const BYTE number_of_bytes_to_overwrite_ = 5;
   private:
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

class MidFunctionHook : public Hook32 {
   public:
    static const BYTE minimum_number_of_bytes_to_overwrite_ = 5;

   private:
    void* original_bytes_;
    void* executeable_buffer_;
    unsigned int hook_address_;
    

   public:
    bool Hook(DWORD source, DWORD destination, unsigned int hook_length);
    bool UnHook(void);
    MidFunctionHook(DWORD source, DWORD destination, unsigned int hook_length);
    //DWORD GetHookAddress(void);
};

}  // namespace Hook32

namespace Hook64 {
class Hook64 {
   protected:
    bool hooked_ = false;

   public:
    bool IsHooked(void);
};

class JumpHook : public Hook64 {
   public:
    static const BYTE number_of_bytes_to_overwrite_ = 5;

   private:
    BYTE original_bytes_[number_of_bytes_to_overwrite_];
    DWORD hook_address_;

   public:
    bool Hook(DWORD source, DWORD destination);
    bool UnHook(void);
    JumpHook(DWORD source, DWORD destination);
    DWORD GetHookAddress(void);
};

class VMTHook : public Hook64 {
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

class MidFunctionHook : public Hook64 {
   public:
    static const BYTE minimum_number_of_bytes_to_overwrite_ = 12;

   private:
    void* original_bytes_;
    void* executeable_buffer_;
    unsigned int hook_address_;

   public:
    bool Hook(DWORD source, DWORD destination, unsigned int hook_length);
    bool UnHook(void);
    MidFunctionHook(DWORD source, DWORD destination, unsigned int hook_length);
    // DWORD GetHookAddress(void);
};

}  // namespace Hook32