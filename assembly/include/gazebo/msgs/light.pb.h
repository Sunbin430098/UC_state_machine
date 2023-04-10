// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: light.proto

#ifndef PROTOBUF_light_2eproto__INCLUDED
#define PROTOBUF_light_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "pose.pb.h"
#include "vector3d.pb.h"
#include "color.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_light_2eproto();
void protobuf_AssignDesc_light_2eproto();
void protobuf_ShutdownFile_light_2eproto();

class Light;

enum Light_LightType {
  Light_LightType_POINT = 1,
  Light_LightType_SPOT = 2,
  Light_LightType_DIRECTIONAL = 3
};
GZ_MSGS_VISIBLE bool Light_LightType_IsValid(int value);
const Light_LightType Light_LightType_LightType_MIN = Light_LightType_POINT;
const Light_LightType Light_LightType_LightType_MAX = Light_LightType_DIRECTIONAL;
const int Light_LightType_LightType_ARRAYSIZE = Light_LightType_LightType_MAX + 1;

GZ_MSGS_VISIBLE const ::google::protobuf::EnumDescriptor* Light_LightType_descriptor();
inline const ::std::string& Light_LightType_Name(Light_LightType value) {
  return ::google::protobuf::internal::NameOfEnum(
    Light_LightType_descriptor(), value);
}
inline bool Light_LightType_Parse(
    const ::std::string& name, Light_LightType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Light_LightType>(
    Light_LightType_descriptor(), name, value);
}
// ===================================================================

class GZ_MSGS_VISIBLE Light : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Light) */ {
 public:
  Light();
  virtual ~Light();

  Light(const Light& from);

  inline Light& operator=(const Light& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Light& default_instance();

  void Swap(Light* other);

  // implements Message ----------------------------------------------

  inline Light* New() const { return New(NULL); }

  Light* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Light& from);
  void MergeFrom(const Light& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Light* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef Light_LightType LightType;
  static const LightType POINT =
    Light_LightType_POINT;
  static const LightType SPOT =
    Light_LightType_SPOT;
  static const LightType DIRECTIONAL =
    Light_LightType_DIRECTIONAL;
  static inline bool LightType_IsValid(int value) {
    return Light_LightType_IsValid(value);
  }
  static const LightType LightType_MIN =
    Light_LightType_LightType_MIN;
  static const LightType LightType_MAX =
    Light_LightType_LightType_MAX;
  static const int LightType_ARRAYSIZE =
    Light_LightType_LightType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  LightType_descriptor() {
    return Light_LightType_descriptor();
  }
  static inline const ::std::string& LightType_Name(LightType value) {
    return Light_LightType_Name(value);
  }
  static inline bool LightType_Parse(const ::std::string& name,
      LightType* value) {
    return Light_LightType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // required string name = 1;
  bool has_name() const;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // optional .gazebo.msgs.Light.LightType type = 2;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  ::gazebo::msgs::Light_LightType type() const;
  void set_type(::gazebo::msgs::Light_LightType value);

  // optional .gazebo.msgs.Pose pose = 3;
  bool has_pose() const;
  void clear_pose();
  static const int kPoseFieldNumber = 3;
  const ::gazebo::msgs::Pose& pose() const;
  ::gazebo::msgs::Pose* mutable_pose();
  ::gazebo::msgs::Pose* release_pose();
  void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // optional .gazebo.msgs.Color diffuse = 4;
  bool has_diffuse() const;
  void clear_diffuse();
  static const int kDiffuseFieldNumber = 4;
  const ::gazebo::msgs::Color& diffuse() const;
  ::gazebo::msgs::Color* mutable_diffuse();
  ::gazebo::msgs::Color* release_diffuse();
  void set_allocated_diffuse(::gazebo::msgs::Color* diffuse);

  // optional .gazebo.msgs.Color specular = 5;
  bool has_specular() const;
  void clear_specular();
  static const int kSpecularFieldNumber = 5;
  const ::gazebo::msgs::Color& specular() const;
  ::gazebo::msgs::Color* mutable_specular();
  ::gazebo::msgs::Color* release_specular();
  void set_allocated_specular(::gazebo::msgs::Color* specular);

  // optional float attenuation_constant = 6;
  bool has_attenuation_constant() const;
  void clear_attenuation_constant();
  static const int kAttenuationConstantFieldNumber = 6;
  float attenuation_constant() const;
  void set_attenuation_constant(float value);

  // optional float attenuation_linear = 7;
  bool has_attenuation_linear() const;
  void clear_attenuation_linear();
  static const int kAttenuationLinearFieldNumber = 7;
  float attenuation_linear() const;
  void set_attenuation_linear(float value);

  // optional float attenuation_quadratic = 8;
  bool has_attenuation_quadratic() const;
  void clear_attenuation_quadratic();
  static const int kAttenuationQuadraticFieldNumber = 8;
  float attenuation_quadratic() const;
  void set_attenuation_quadratic(float value);

  // optional .gazebo.msgs.Vector3d direction = 9;
  bool has_direction() const;
  void clear_direction();
  static const int kDirectionFieldNumber = 9;
  const ::gazebo::msgs::Vector3d& direction() const;
  ::gazebo::msgs::Vector3d* mutable_direction();
  ::gazebo::msgs::Vector3d* release_direction();
  void set_allocated_direction(::gazebo::msgs::Vector3d* direction);

  // optional float range = 10;
  bool has_range() const;
  void clear_range();
  static const int kRangeFieldNumber = 10;
  float range() const;
  void set_range(float value);

  // optional bool cast_shadows = 11;
  bool has_cast_shadows() const;
  void clear_cast_shadows();
  static const int kCastShadowsFieldNumber = 11;
  bool cast_shadows() const;
  void set_cast_shadows(bool value);

  // optional float spot_inner_angle = 12;
  bool has_spot_inner_angle() const;
  void clear_spot_inner_angle();
  static const int kSpotInnerAngleFieldNumber = 12;
  float spot_inner_angle() const;
  void set_spot_inner_angle(float value);

  // optional float spot_outer_angle = 13;
  bool has_spot_outer_angle() const;
  void clear_spot_outer_angle();
  static const int kSpotOuterAngleFieldNumber = 13;
  float spot_outer_angle() const;
  void set_spot_outer_angle(float value);

  // optional float spot_falloff = 14;
  bool has_spot_falloff() const;
  void clear_spot_falloff();
  static const int kSpotFalloffFieldNumber = 14;
  float spot_falloff() const;
  void set_spot_falloff(float value);

  // optional uint32 id = 15;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 15;
  ::google::protobuf::uint32 id() const;
  void set_id(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Light)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_diffuse();
  inline void clear_has_diffuse();
  inline void set_has_specular();
  inline void clear_has_specular();
  inline void set_has_attenuation_constant();
  inline void clear_has_attenuation_constant();
  inline void set_has_attenuation_linear();
  inline void clear_has_attenuation_linear();
  inline void set_has_attenuation_quadratic();
  inline void clear_has_attenuation_quadratic();
  inline void set_has_direction();
  inline void clear_has_direction();
  inline void set_has_range();
  inline void clear_has_range();
  inline void set_has_cast_shadows();
  inline void clear_has_cast_shadows();
  inline void set_has_spot_inner_angle();
  inline void clear_has_spot_inner_angle();
  inline void set_has_spot_outer_angle();
  inline void clear_has_spot_outer_angle();
  inline void set_has_spot_falloff();
  inline void clear_has_spot_falloff();
  inline void set_has_id();
  inline void clear_has_id();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  ::gazebo::msgs::Pose* pose_;
  ::gazebo::msgs::Color* diffuse_;
  int type_;
  float attenuation_constant_;
  ::gazebo::msgs::Color* specular_;
  float attenuation_linear_;
  float attenuation_quadratic_;
  ::gazebo::msgs::Vector3d* direction_;
  float range_;
  bool cast_shadows_;
  float spot_inner_angle_;
  float spot_outer_angle_;
  float spot_falloff_;
  ::google::protobuf::uint32 id_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_light_2eproto();
  friend void protobuf_AssignDesc_light_2eproto();
  friend void protobuf_ShutdownFile_light_2eproto();

  void InitAsDefaultInstance();
  static Light* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Light

// required string name = 1;
inline bool Light::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Light::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Light::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Light::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& Light::name() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.name)
  return name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Light::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.name)
}
inline void Light::set_name(const char* value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Light.name)
}
inline void Light::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Light.name)
}
inline ::std::string* Light::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Light.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Light::release_name() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Light.name)
  clear_has_name();
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Light::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Light.name)
}

// optional .gazebo.msgs.Light.LightType type = 2;
inline bool Light::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Light::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Light::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Light::clear_type() {
  type_ = 1;
  clear_has_type();
}
inline ::gazebo::msgs::Light_LightType Light::type() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.type)
  return static_cast< ::gazebo::msgs::Light_LightType >(type_);
}
inline void Light::set_type(::gazebo::msgs::Light_LightType value) {
  assert(::gazebo::msgs::Light_LightType_IsValid(value));
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.type)
}

// optional .gazebo.msgs.Pose pose = 3;
inline bool Light::has_pose() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Light::set_has_pose() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Light::clear_has_pose() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Light::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& Light::pose() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.pose)
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* Light::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) {
    pose_ = new ::gazebo::msgs::Pose;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Light.pose)
  return pose_;
}
inline ::gazebo::msgs::Pose* Light::release_pose() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Light.pose)
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void Light::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Light.pose)
}

// optional .gazebo.msgs.Color diffuse = 4;
inline bool Light::has_diffuse() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Light::set_has_diffuse() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Light::clear_has_diffuse() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Light::clear_diffuse() {
  if (diffuse_ != NULL) diffuse_->::gazebo::msgs::Color::Clear();
  clear_has_diffuse();
}
inline const ::gazebo::msgs::Color& Light::diffuse() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.diffuse)
  return diffuse_ != NULL ? *diffuse_ : *default_instance_->diffuse_;
}
inline ::gazebo::msgs::Color* Light::mutable_diffuse() {
  set_has_diffuse();
  if (diffuse_ == NULL) {
    diffuse_ = new ::gazebo::msgs::Color;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Light.diffuse)
  return diffuse_;
}
inline ::gazebo::msgs::Color* Light::release_diffuse() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Light.diffuse)
  clear_has_diffuse();
  ::gazebo::msgs::Color* temp = diffuse_;
  diffuse_ = NULL;
  return temp;
}
inline void Light::set_allocated_diffuse(::gazebo::msgs::Color* diffuse) {
  delete diffuse_;
  diffuse_ = diffuse;
  if (diffuse) {
    set_has_diffuse();
  } else {
    clear_has_diffuse();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Light.diffuse)
}

// optional .gazebo.msgs.Color specular = 5;
inline bool Light::has_specular() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Light::set_has_specular() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Light::clear_has_specular() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Light::clear_specular() {
  if (specular_ != NULL) specular_->::gazebo::msgs::Color::Clear();
  clear_has_specular();
}
inline const ::gazebo::msgs::Color& Light::specular() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.specular)
  return specular_ != NULL ? *specular_ : *default_instance_->specular_;
}
inline ::gazebo::msgs::Color* Light::mutable_specular() {
  set_has_specular();
  if (specular_ == NULL) {
    specular_ = new ::gazebo::msgs::Color;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Light.specular)
  return specular_;
}
inline ::gazebo::msgs::Color* Light::release_specular() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Light.specular)
  clear_has_specular();
  ::gazebo::msgs::Color* temp = specular_;
  specular_ = NULL;
  return temp;
}
inline void Light::set_allocated_specular(::gazebo::msgs::Color* specular) {
  delete specular_;
  specular_ = specular;
  if (specular) {
    set_has_specular();
  } else {
    clear_has_specular();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Light.specular)
}

// optional float attenuation_constant = 6;
inline bool Light::has_attenuation_constant() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Light::set_has_attenuation_constant() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Light::clear_has_attenuation_constant() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Light::clear_attenuation_constant() {
  attenuation_constant_ = 0;
  clear_has_attenuation_constant();
}
inline float Light::attenuation_constant() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.attenuation_constant)
  return attenuation_constant_;
}
inline void Light::set_attenuation_constant(float value) {
  set_has_attenuation_constant();
  attenuation_constant_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.attenuation_constant)
}

// optional float attenuation_linear = 7;
inline bool Light::has_attenuation_linear() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Light::set_has_attenuation_linear() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Light::clear_has_attenuation_linear() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Light::clear_attenuation_linear() {
  attenuation_linear_ = 0;
  clear_has_attenuation_linear();
}
inline float Light::attenuation_linear() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.attenuation_linear)
  return attenuation_linear_;
}
inline void Light::set_attenuation_linear(float value) {
  set_has_attenuation_linear();
  attenuation_linear_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.attenuation_linear)
}

// optional float attenuation_quadratic = 8;
inline bool Light::has_attenuation_quadratic() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Light::set_has_attenuation_quadratic() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Light::clear_has_attenuation_quadratic() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Light::clear_attenuation_quadratic() {
  attenuation_quadratic_ = 0;
  clear_has_attenuation_quadratic();
}
inline float Light::attenuation_quadratic() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.attenuation_quadratic)
  return attenuation_quadratic_;
}
inline void Light::set_attenuation_quadratic(float value) {
  set_has_attenuation_quadratic();
  attenuation_quadratic_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.attenuation_quadratic)
}

// optional .gazebo.msgs.Vector3d direction = 9;
inline bool Light::has_direction() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void Light::set_has_direction() {
  _has_bits_[0] |= 0x00000100u;
}
inline void Light::clear_has_direction() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void Light::clear_direction() {
  if (direction_ != NULL) direction_->::gazebo::msgs::Vector3d::Clear();
  clear_has_direction();
}
inline const ::gazebo::msgs::Vector3d& Light::direction() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.direction)
  return direction_ != NULL ? *direction_ : *default_instance_->direction_;
}
inline ::gazebo::msgs::Vector3d* Light::mutable_direction() {
  set_has_direction();
  if (direction_ == NULL) {
    direction_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Light.direction)
  return direction_;
}
inline ::gazebo::msgs::Vector3d* Light::release_direction() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Light.direction)
  clear_has_direction();
  ::gazebo::msgs::Vector3d* temp = direction_;
  direction_ = NULL;
  return temp;
}
inline void Light::set_allocated_direction(::gazebo::msgs::Vector3d* direction) {
  delete direction_;
  direction_ = direction;
  if (direction) {
    set_has_direction();
  } else {
    clear_has_direction();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Light.direction)
}

// optional float range = 10;
inline bool Light::has_range() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void Light::set_has_range() {
  _has_bits_[0] |= 0x00000200u;
}
inline void Light::clear_has_range() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void Light::clear_range() {
  range_ = 0;
  clear_has_range();
}
inline float Light::range() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.range)
  return range_;
}
inline void Light::set_range(float value) {
  set_has_range();
  range_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.range)
}

// optional bool cast_shadows = 11;
inline bool Light::has_cast_shadows() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void Light::set_has_cast_shadows() {
  _has_bits_[0] |= 0x00000400u;
}
inline void Light::clear_has_cast_shadows() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void Light::clear_cast_shadows() {
  cast_shadows_ = false;
  clear_has_cast_shadows();
}
inline bool Light::cast_shadows() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.cast_shadows)
  return cast_shadows_;
}
inline void Light::set_cast_shadows(bool value) {
  set_has_cast_shadows();
  cast_shadows_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.cast_shadows)
}

// optional float spot_inner_angle = 12;
inline bool Light::has_spot_inner_angle() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void Light::set_has_spot_inner_angle() {
  _has_bits_[0] |= 0x00000800u;
}
inline void Light::clear_has_spot_inner_angle() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void Light::clear_spot_inner_angle() {
  spot_inner_angle_ = 0;
  clear_has_spot_inner_angle();
}
inline float Light::spot_inner_angle() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.spot_inner_angle)
  return spot_inner_angle_;
}
inline void Light::set_spot_inner_angle(float value) {
  set_has_spot_inner_angle();
  spot_inner_angle_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.spot_inner_angle)
}

// optional float spot_outer_angle = 13;
inline bool Light::has_spot_outer_angle() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void Light::set_has_spot_outer_angle() {
  _has_bits_[0] |= 0x00001000u;
}
inline void Light::clear_has_spot_outer_angle() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void Light::clear_spot_outer_angle() {
  spot_outer_angle_ = 0;
  clear_has_spot_outer_angle();
}
inline float Light::spot_outer_angle() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.spot_outer_angle)
  return spot_outer_angle_;
}
inline void Light::set_spot_outer_angle(float value) {
  set_has_spot_outer_angle();
  spot_outer_angle_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.spot_outer_angle)
}

// optional float spot_falloff = 14;
inline bool Light::has_spot_falloff() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void Light::set_has_spot_falloff() {
  _has_bits_[0] |= 0x00002000u;
}
inline void Light::clear_has_spot_falloff() {
  _has_bits_[0] &= ~0x00002000u;
}
inline void Light::clear_spot_falloff() {
  spot_falloff_ = 0;
  clear_has_spot_falloff();
}
inline float Light::spot_falloff() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.spot_falloff)
  return spot_falloff_;
}
inline void Light::set_spot_falloff(float value) {
  set_has_spot_falloff();
  spot_falloff_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.spot_falloff)
}

// optional uint32 id = 15;
inline bool Light::has_id() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void Light::set_has_id() {
  _has_bits_[0] |= 0x00004000u;
}
inline void Light::clear_has_id() {
  _has_bits_[0] &= ~0x00004000u;
}
inline void Light::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 Light::id() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Light.id)
  return id_;
}
inline void Light::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Light.id)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Light> LightPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::gazebo::msgs::Light_LightType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gazebo::msgs::Light_LightType>() {
  return ::gazebo::msgs::Light_LightType_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Light const> ConstLightPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_light_2eproto__INCLUDED