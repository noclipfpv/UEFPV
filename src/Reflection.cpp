// Reflection.cpp - see Reflection.hpp
#include "Reflection.hpp"

#include <cstring>
#include <malloc.h>

namespace fpv::refl {

API::FProperty* find_property(API::UStruct* s, std::wstring_view name) {
    for (auto c = s; c != nullptr; c = c->get_super()) {
        if (auto p = c->find_property(name); p != nullptr) return p;
    }
    return nullptr;
}

API::UFunction* find_function(API::UStruct* s, std::wstring_view name) {
    for (auto c = s; c != nullptr; c = c->get_super()) {
        if (auto f = c->find_function(name); f != nullptr) return f;
    }
    return nullptr;
}

int32_t struct_size(API::FProperty* prop) {
    if (prop == nullptr) return 0;
    const auto s = static_cast<API::FStructProperty*>(prop)->get_struct();
    return s != nullptr ? s->get_struct_size() : 0;
}

namespace {

template <typename T>
void put3(uint8_t* dst, double a, double b, double c) {
    const T tmp[3]{static_cast<T>(a), static_cast<T>(b), static_cast<T>(c)};
    std::memcpy(dst, tmp, sizeof(tmp));
}

template <typename T>
void put4(uint8_t* dst, double a, double b, double c, double d) {
    const T tmp[4]{static_cast<T>(a), static_cast<T>(b), static_cast<T>(c), static_cast<T>(d)};
    std::memcpy(dst, tmp, sizeof(tmp));
}

bool write3(uint8_t* dst, API::FProperty* prop, double a, double b, double c) {
    const auto size = struct_size(prop);
    if (size >= 24) {
        put3<double>(dst, a, b, c);
    } else if (size >= 12) {
        put3<float>(dst, a, b, c);
    } else {
        return false;
    }
    return true;
}

} // namespace

bool write_vector(uint8_t* dst, API::FProperty* prop, const Vec3& v) {
    return dst != nullptr && write3(dst, prop, v.x, v.y, v.z);
}

bool write_rotator(uint8_t* dst, API::FProperty* prop, const Rotator& r) {
    // FRotator memory order: Pitch, Yaw, Roll
    return dst != nullptr && write3(dst, prop, r.pitch, r.yaw, r.roll);
}

Vec3 read_vector(const uint8_t* src, API::FProperty* prop) {
    if (src == nullptr) return {};
    if (struct_size(prop) >= 24) {
        double t[3]{};
        std::memcpy(t, src, sizeof(t));
        return {t[0], t[1], t[2]};
    }
    float t[3]{};
    std::memcpy(t, src, sizeof(t));
    return {t[0], t[1], t[2]};
}

bool write_transform(uint8_t* dst, API::FProperty* prop, const Vec3& translation) {
    if (dst == nullptr || prop == nullptr) return false;
    const auto s = static_cast<API::FStructProperty*>(prop)->get_struct();
    if (s == nullptr) return false;

    const auto rot = find_property(s, L"Rotation");
    const auto loc = find_property(s, L"Translation");
    const auto scale = find_property(s, L"Scale3D");
    if (rot == nullptr || loc == nullptr || scale == nullptr) return false;

    uint8_t* q = dst + rot->get_offset();
    if (struct_size(rot) >= 32) {
        put4<double>(q, 0, 0, 0, 1);
    } else {
        put4<float>(q, 0, 0, 0, 1);
    }
    return write3(dst + loc->get_offset(), loc, translation.x, translation.y, translation.z) &&
           write3(dst + scale->get_offset(), scale, 1.0, 1.0, 1.0);
}

API::UObject* get_object_property(API::UObject* obj, std::wstring_view name) {
    if (obj == nullptr) return nullptr;
    const auto p = obj->get_property_data<API::UObject*>(name);
    return p != nullptr ? *p : nullptr;
}

Call::Call(API::UObject* target, std::wstring_view function_name) : m_target{target} {
    if (target == nullptr) return;
    m_function = find_function(target->get_class(), function_name);
    if (m_function == nullptr) return;

    const auto size = static_cast<size_t>(std::max(m_function->get_properties_size(), 1));
    m_params = static_cast<uint8_t*>(_aligned_malloc(size + 32, 32));
    if (m_params != nullptr) std::memset(m_params, 0, size + 32);
}

Call::~Call() {
    if (m_params != nullptr) _aligned_free(m_params);
}

uint8_t* Call::param(std::wstring_view name, API::FProperty** out_prop) const {
    if (!valid()) return nullptr;
    auto prop = m_function->find_property(name);
    if (out_prop != nullptr) *out_prop = prop;
    return prop != nullptr ? m_params + prop->get_offset() : nullptr;
}

bool Call::set_vector(std::wstring_view name, const Vec3& v) {
    API::FProperty* prop{};
    auto p = param(name, &prop);
    return write_vector(p, prop, v);
}

bool Call::set_rotator(std::wstring_view name, const Rotator& r) {
    API::FProperty* prop{};
    auto p = param(name, &prop);
    return write_rotator(p, prop, r);
}

bool Call::set_transform(std::wstring_view name, const Vec3& translation) {
    API::FProperty* prop{};
    auto p = param(name, &prop);
    return write_transform(p, prop, translation);
}

bool Call::invoke() {
    if (!valid()) return false;
    m_target->process_event(m_function, m_params);
    return true;
}

} // namespace fpv::refl
