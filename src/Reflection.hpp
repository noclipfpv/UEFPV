// Reflection.hpp - helpers to call Unreal UFunctions generically through UEVR's API.
//
// Parameter buffers are laid out from the UFunction's own reflection data (offsets and
// sizes), so the same code works across UE4 / UE5 and float / double vector builds.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "uevr/API.hpp"

#include "MathTypes.hpp"

namespace fpv::refl {

using uevr::API;

// Searches the struct and all of its super structs.
API::FProperty* find_property(API::UStruct* s, std::wstring_view name);
API::UFunction* find_function(API::UStruct* s, std::wstring_view name);

// Size of the UScriptStruct behind a struct property (0 if unknown).
int32_t struct_size(API::FProperty* prop);

// Writes an FVector / FRotator (float or double layout, detected from the struct size).
bool write_vector(uint8_t* dst, API::FProperty* prop, const Vec3& v);
bool write_rotator(uint8_t* dst, API::FProperty* prop, const Rotator& r);
// Reads an FVector-like struct (float or double layout).
Vec3 read_vector(const uint8_t* src, API::FProperty* prop);
// Writes an FTransform with identity rotation, the given translation and unit scale.
bool write_transform(uint8_t* dst, API::FProperty* prop, const Vec3& translation);

// Reads a UObject* property of an object (nullptr if missing).
API::UObject* get_object_property(API::UObject* obj, std::wstring_view name);

// One UFunction call with a zero-initialized parameter buffer.
class Call {
public:
    Call(API::UObject* target, std::wstring_view function_name);
    ~Call();
    Call(const Call&) = delete;
    Call& operator=(const Call&) = delete;

    bool valid() const { return m_target != nullptr && m_function != nullptr && m_params != nullptr; }

    // Raw access to a parameter (nullptr if the parameter does not exist).
    uint8_t* param(std::wstring_view name, API::FProperty** out_prop = nullptr) const;

    template <typename T>
    bool set(std::wstring_view name, const T& value) {
        auto p = param(name);
        if (p == nullptr) return false;
        std::memcpy(p, &value, sizeof(T));
        return true;
    }

    bool set_vector(std::wstring_view name, const Vec3& v);
    bool set_rotator(std::wstring_view name, const Rotator& r);
    bool set_transform(std::wstring_view name, const Vec3& translation);

    template <typename T>
    T get(std::wstring_view name) const {
        T out{};
        if (auto p = param(name)) std::memcpy(&out, p, sizeof(T));
        return out;
    }

    // Calls the function through ProcessEvent. Returns false if the call could not be made.
    bool invoke();

private:
    API::UObject* m_target{nullptr};
    API::UFunction* m_function{nullptr};
    uint8_t* m_params{nullptr};
};

} // namespace fpv::refl
