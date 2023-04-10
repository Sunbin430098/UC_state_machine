// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: fog.proto

#ifndef PROTOBUF_fog_2eproto__INCLUDED
#define PROTOBUF_fog_2eproto__INCLUDED

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
#include "color.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_fog_2eproto();
void protobuf_AssignDesc_fog_2eproto();
void protobuf_ShutdownFile_fog_2eproto();

class Fog;

enum Fog_FogType {
  Fog_FogType_NONE = 1,
  Fog_FogType_LINEAR = 2,
  Fog_FogType_EXPONENTIAL = 3,
  Fog_FogType_EXPONENTIAL2 = 4
};
GZ_MSGS_VISIBLE bool Fog_FogType_IsValid(int value);
const Fog_FogType Fog_FogType_FogType_MIN = Fog_FogType_NONE;
const Fog_FogType Fog_FogType_FogType_MAX = Fog_FogType_EXPONENTIAL2;
const int Fog_FogType_FogType_ARRAYSIZE = Fog_FogType_FogType_MAX + 1;

GZ_MSGS_VISIBLE const ::google::protobuf::EnumDescriptor* Fog_FogType_descriptor();
inline const ::std::string& Fog_FogType_Name(Fog_FogType value) {
  return ::google::protobuf::internal::NameOfEnum(
    Fog_FogType_descriptor(), value);
}
inline bool Fog_FogType_Parse(
    const ::std::string& name, Fog_FogType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Fog_FogType>(
    Fog_FogType_descriptor(), name, value);
}
// ===================================================================

class GZ_MSGS_VISIBLE Fog : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Fog) */ {
 public:
  Fog();
  virtual ~Fog();

  Fog(const Fog& from);

  inline Fog& operator=(const Fog& from) {
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
  static const Fog& default_instance();

  void Swap(Fog* other);

  // implements Message ----------------------------------------------

  inline Fog* New() const { return New(NULL); }

  Fog* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Fog& from);
  void MergeFrom(const Fog& from);
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
  void InternalSwap(Fog* other);
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

  typedef Fog_FogType FogType;
  static const FogType NONE =
    Fog_FogType_NONE;
  static const FogType LINEAR =
    Fog_FogType_LINEAR;
  static const FogType EXPONENTIAL =
    Fog_FogType_EXPONENTIAL;
  static const FogType EXPONENTIAL2 =
    Fog_FogType_EXPONENTIAL2;
  static inline bool FogType_IsValid(int value) {
    return Fog_FogType_IsValid(value);
  }
  static const FogType FogType_MIN =
    Fog_FogType_FogType_MIN;
  static const FogType FogType_MAX =
    Fog_FogType_FogType_MAX;
  static const int FogType_ARRAYSIZE =
    Fog_FogType_FogType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  FogType_descriptor() {
    return Fog_FogType_descriptor();
  }
  static inline const ::std::string& FogType_Name(FogType value) {
    return Fog_FogType_Name(value);
  }
  static inline bool FogType_Parse(const ::std::string& name,
      FogType* value) {
    return Fog_FogType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional .gazebo.msgs.Fog.FogType type = 1;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 1;
  ::gazebo::msgs::Fog_FogType type() const;
  void set_type(::gazebo::msgs::Fog_FogType value);

  // optional .gazebo.msgs.Color color = 2;
  bool has_color() const;
  void clear_color();
  static const int kColorFieldNumber = 2;
  const ::gazebo::msgs::Color& color() const;
  ::gazebo::msgs::Color* mutable_color();
  ::gazebo::msgs::Color* release_color();
  void set_allocated_color(::gazebo::msgs::Color* color);

  // optional float density = 3;
  bool has_density() const;
  void clear_density();
  static const int kDensityFieldNumber = 3;
  float density() const;
  void set_density(float value);

  // optional float start = 4;
  bool has_start() const;
  void clear_start();
  static const int kStartFieldNumber = 4;
  float start() const;
  void set_start(float value);

  // optional float end = 5;
  bool has_end() const;
  void clear_end();
  static const int kEndFieldNumber = 5;
  float end() const;
  void set_end(float value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Fog)
 private:
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_color();
  inline void clear_has_color();
  inline void set_has_density();
  inline void clear_has_density();
  inline void set_has_start();
  inline void clear_has_start();
  inline void set_has_end();
  inline void clear_has_end();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Color* color_;
  int type_;
  float density_;
  float start_;
  float end_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_fog_2eproto();
  friend void protobuf_AssignDesc_fog_2eproto();
  friend void protobuf_ShutdownFile_fog_2eproto();

  void InitAsDefaultInstance();
  static Fog* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Fog

// optional .gazebo.msgs.Fog.FogType type = 1;
inline bool Fog::has_type() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Fog::set_has_type() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Fog::clear_has_type() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Fog::clear_type() {
  type_ = 1;
  clear_has_type();
}
inline ::gazebo::msgs::Fog_FogType Fog::type() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Fog.type)
  return static_cast< ::gazebo::msgs::Fog_FogType >(type_);
}
inline void Fog::set_type(::gazebo::msgs::Fog_FogType value) {
  assert(::gazebo::msgs::Fog_FogType_IsValid(value));
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Fog.type)
}

// optional .gazebo.msgs.Color color = 2;
inline bool Fog::has_color() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Fog::set_has_color() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Fog::clear_has_color() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Fog::clear_color() {
  if (color_ != NULL) color_->::gazebo::msgs::Color::Clear();
  clear_has_color();
}
inline const ::gazebo::msgs::Color& Fog::color() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Fog.color)
  return color_ != NULL ? *color_ : *default_instance_->color_;
}
inline ::gazebo::msgs::Color* Fog::mutable_color() {
  set_has_color();
  if (color_ == NULL) {
    color_ = new ::gazebo::msgs::Color;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Fog.color)
  return color_;
}
inline ::gazebo::msgs::Color* Fog::release_color() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Fog.color)
  clear_has_color();
  ::gazebo::msgs::Color* temp = color_;
  color_ = NULL;
  return temp;
}
inline void Fog::set_allocated_color(::gazebo::msgs::Color* color) {
  delete color_;
  color_ = color;
  if (color) {
    set_has_color();
  } else {
    clear_has_color();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Fog.color)
}

// optional float density = 3;
inline bool Fog::has_density() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Fog::set_has_density() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Fog::clear_has_density() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Fog::clear_density() {
  density_ = 0;
  clear_has_density();
}
inline float Fog::density() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Fog.density)
  return density_;
}
inline void Fog::set_density(float value) {
  set_has_density();
  density_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Fog.density)
}

// optional float start = 4;
inline bool Fog::has_start() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Fog::set_has_start() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Fog::clear_has_start() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Fog::clear_start() {
  start_ = 0;
  clear_has_start();
}
inline float Fog::start() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Fog.start)
  return start_;
}
inline void Fog::set_start(float value) {
  set_has_start();
  start_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Fog.start)
}

// optional float end = 5;
inline bool Fog::has_end() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Fog::set_has_end() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Fog::clear_has_end() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Fog::clear_end() {
  end_ = 0;
  clear_has_end();
}
inline float Fog::end() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Fog.end)
  return end_;
}
inline void Fog::set_end(float value) {
  set_has_end();
  end_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Fog.end)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Fog> FogPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::gazebo::msgs::Fog_FogType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gazebo::msgs::Fog_FogType>() {
  return ::gazebo::msgs::Fog_FogType_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Fog const> ConstFogPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_fog_2eproto__INCLUDED