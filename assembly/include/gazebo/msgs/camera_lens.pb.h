// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: camera_lens.proto

#ifndef PROTOBUF_camera_5flens_2eproto__INCLUDED
#define PROTOBUF_camera_5flens_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_camera_5flens_2eproto();
void protobuf_AssignDesc_camera_5flens_2eproto();
void protobuf_ShutdownFile_camera_5flens_2eproto();

class CameraLens;

// ===================================================================

class GZ_MSGS_VISIBLE CameraLens : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.CameraLens) */ {
 public:
  CameraLens();
  virtual ~CameraLens();

  CameraLens(const CameraLens& from);

  inline CameraLens& operator=(const CameraLens& from) {
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
  static const CameraLens& default_instance();

  void Swap(CameraLens* other);

  // implements Message ----------------------------------------------

  inline CameraLens* New() const { return New(NULL); }

  CameraLens* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const CameraLens& from);
  void MergeFrom(const CameraLens& from);
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
  void InternalSwap(CameraLens* other);
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

  // accessors -------------------------------------------------------

  // required string type = 1;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 1;
  const ::std::string& type() const;
  void set_type(const ::std::string& value);
  void set_type(const char* value);
  void set_type(const char* value, size_t size);
  ::std::string* mutable_type();
  ::std::string* release_type();
  void set_allocated_type(::std::string* type);

  // optional double c1 = 2;
  bool has_c1() const;
  void clear_c1();
  static const int kC1FieldNumber = 2;
  double c1() const;
  void set_c1(double value);

  // optional double c2 = 3;
  bool has_c2() const;
  void clear_c2();
  static const int kC2FieldNumber = 3;
  double c2() const;
  void set_c2(double value);

  // optional double c3 = 4;
  bool has_c3() const;
  void clear_c3();
  static const int kC3FieldNumber = 4;
  double c3() const;
  void set_c3(double value);

  // optional double f = 5;
  bool has_f() const;
  void clear_f();
  static const int kFFieldNumber = 5;
  double f() const;
  void set_f(double value);

  // optional string fun = 6;
  bool has_fun() const;
  void clear_fun();
  static const int kFunFieldNumber = 6;
  const ::std::string& fun() const;
  void set_fun(const ::std::string& value);
  void set_fun(const char* value);
  void set_fun(const char* value, size_t size);
  ::std::string* mutable_fun();
  ::std::string* release_fun();
  void set_allocated_fun(::std::string* fun);

  // optional bool scale_to_hfov = 7;
  bool has_scale_to_hfov() const;
  void clear_scale_to_hfov();
  static const int kScaleToHfovFieldNumber = 7;
  bool scale_to_hfov() const;
  void set_scale_to_hfov(bool value);

  // optional double cutoff_angle = 8;
  bool has_cutoff_angle() const;
  void clear_cutoff_angle();
  static const int kCutoffAngleFieldNumber = 8;
  double cutoff_angle() const;
  void set_cutoff_angle(double value);

  // optional double hfov = 9;
  bool has_hfov() const;
  void clear_hfov();
  static const int kHfovFieldNumber = 9;
  double hfov() const;
  void set_hfov(double value);

  // optional int32 env_texture_size = 10;
  bool has_env_texture_size() const;
  void clear_env_texture_size();
  static const int kEnvTextureSizeFieldNumber = 10;
  ::google::protobuf::int32 env_texture_size() const;
  void set_env_texture_size(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.CameraLens)
 private:
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_c1();
  inline void clear_has_c1();
  inline void set_has_c2();
  inline void clear_has_c2();
  inline void set_has_c3();
  inline void clear_has_c3();
  inline void set_has_f();
  inline void clear_has_f();
  inline void set_has_fun();
  inline void clear_has_fun();
  inline void set_has_scale_to_hfov();
  inline void clear_has_scale_to_hfov();
  inline void set_has_cutoff_angle();
  inline void clear_has_cutoff_angle();
  inline void set_has_hfov();
  inline void clear_has_hfov();
  inline void set_has_env_texture_size();
  inline void clear_has_env_texture_size();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr type_;
  double c1_;
  double c2_;
  double c3_;
  double f_;
  ::google::protobuf::internal::ArenaStringPtr fun_;
  double cutoff_angle_;
  bool scale_to_hfov_;
  ::google::protobuf::int32 env_texture_size_;
  double hfov_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_camera_5flens_2eproto();
  friend void protobuf_AssignDesc_camera_5flens_2eproto();
  friend void protobuf_ShutdownFile_camera_5flens_2eproto();

  void InitAsDefaultInstance();
  static CameraLens* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// CameraLens

// required string type = 1;
inline bool CameraLens::has_type() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CameraLens::set_has_type() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CameraLens::clear_has_type() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void CameraLens::clear_type() {
  type_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_type();
}
inline const ::std::string& CameraLens::type() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.type)
  return type_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CameraLens::set_type(const ::std::string& value) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.type)
}
inline void CameraLens::set_type(const char* value) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.CameraLens.type)
}
inline void CameraLens::set_type(const char* value, size_t size) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.CameraLens.type)
}
inline ::std::string* CameraLens::mutable_type() {
  set_has_type();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.CameraLens.type)
  return type_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* CameraLens::release_type() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.CameraLens.type)
  clear_has_type();
  return type_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CameraLens::set_allocated_type(::std::string* type) {
  if (type != NULL) {
    set_has_type();
  } else {
    clear_has_type();
  }
  type_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), type);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.CameraLens.type)
}

// optional double c1 = 2;
inline bool CameraLens::has_c1() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CameraLens::set_has_c1() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CameraLens::clear_has_c1() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CameraLens::clear_c1() {
  c1_ = 0;
  clear_has_c1();
}
inline double CameraLens::c1() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.c1)
  return c1_;
}
inline void CameraLens::set_c1(double value) {
  set_has_c1();
  c1_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.c1)
}

// optional double c2 = 3;
inline bool CameraLens::has_c2() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void CameraLens::set_has_c2() {
  _has_bits_[0] |= 0x00000004u;
}
inline void CameraLens::clear_has_c2() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void CameraLens::clear_c2() {
  c2_ = 0;
  clear_has_c2();
}
inline double CameraLens::c2() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.c2)
  return c2_;
}
inline void CameraLens::set_c2(double value) {
  set_has_c2();
  c2_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.c2)
}

// optional double c3 = 4;
inline bool CameraLens::has_c3() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void CameraLens::set_has_c3() {
  _has_bits_[0] |= 0x00000008u;
}
inline void CameraLens::clear_has_c3() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void CameraLens::clear_c3() {
  c3_ = 0;
  clear_has_c3();
}
inline double CameraLens::c3() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.c3)
  return c3_;
}
inline void CameraLens::set_c3(double value) {
  set_has_c3();
  c3_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.c3)
}

// optional double f = 5;
inline bool CameraLens::has_f() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void CameraLens::set_has_f() {
  _has_bits_[0] |= 0x00000010u;
}
inline void CameraLens::clear_has_f() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void CameraLens::clear_f() {
  f_ = 0;
  clear_has_f();
}
inline double CameraLens::f() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.f)
  return f_;
}
inline void CameraLens::set_f(double value) {
  set_has_f();
  f_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.f)
}

// optional string fun = 6;
inline bool CameraLens::has_fun() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void CameraLens::set_has_fun() {
  _has_bits_[0] |= 0x00000020u;
}
inline void CameraLens::clear_has_fun() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void CameraLens::clear_fun() {
  fun_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_fun();
}
inline const ::std::string& CameraLens::fun() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.fun)
  return fun_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CameraLens::set_fun(const ::std::string& value) {
  set_has_fun();
  fun_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.fun)
}
inline void CameraLens::set_fun(const char* value) {
  set_has_fun();
  fun_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.CameraLens.fun)
}
inline void CameraLens::set_fun(const char* value, size_t size) {
  set_has_fun();
  fun_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.CameraLens.fun)
}
inline ::std::string* CameraLens::mutable_fun() {
  set_has_fun();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.CameraLens.fun)
  return fun_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* CameraLens::release_fun() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.CameraLens.fun)
  clear_has_fun();
  return fun_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CameraLens::set_allocated_fun(::std::string* fun) {
  if (fun != NULL) {
    set_has_fun();
  } else {
    clear_has_fun();
  }
  fun_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), fun);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.CameraLens.fun)
}

// optional bool scale_to_hfov = 7;
inline bool CameraLens::has_scale_to_hfov() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void CameraLens::set_has_scale_to_hfov() {
  _has_bits_[0] |= 0x00000040u;
}
inline void CameraLens::clear_has_scale_to_hfov() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void CameraLens::clear_scale_to_hfov() {
  scale_to_hfov_ = false;
  clear_has_scale_to_hfov();
}
inline bool CameraLens::scale_to_hfov() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.scale_to_hfov)
  return scale_to_hfov_;
}
inline void CameraLens::set_scale_to_hfov(bool value) {
  set_has_scale_to_hfov();
  scale_to_hfov_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.scale_to_hfov)
}

// optional double cutoff_angle = 8;
inline bool CameraLens::has_cutoff_angle() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void CameraLens::set_has_cutoff_angle() {
  _has_bits_[0] |= 0x00000080u;
}
inline void CameraLens::clear_has_cutoff_angle() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void CameraLens::clear_cutoff_angle() {
  cutoff_angle_ = 0;
  clear_has_cutoff_angle();
}
inline double CameraLens::cutoff_angle() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.cutoff_angle)
  return cutoff_angle_;
}
inline void CameraLens::set_cutoff_angle(double value) {
  set_has_cutoff_angle();
  cutoff_angle_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.cutoff_angle)
}

// optional double hfov = 9;
inline bool CameraLens::has_hfov() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void CameraLens::set_has_hfov() {
  _has_bits_[0] |= 0x00000100u;
}
inline void CameraLens::clear_has_hfov() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void CameraLens::clear_hfov() {
  hfov_ = 0;
  clear_has_hfov();
}
inline double CameraLens::hfov() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.hfov)
  return hfov_;
}
inline void CameraLens::set_hfov(double value) {
  set_has_hfov();
  hfov_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.hfov)
}

// optional int32 env_texture_size = 10;
inline bool CameraLens::has_env_texture_size() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void CameraLens::set_has_env_texture_size() {
  _has_bits_[0] |= 0x00000200u;
}
inline void CameraLens::clear_has_env_texture_size() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void CameraLens::clear_env_texture_size() {
  env_texture_size_ = 0;
  clear_has_env_texture_size();
}
inline ::google::protobuf::int32 CameraLens::env_texture_size() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.CameraLens.env_texture_size)
  return env_texture_size_;
}
inline void CameraLens::set_env_texture_size(::google::protobuf::int32 value) {
  set_has_env_texture_size();
  env_texture_size_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.CameraLens.env_texture_size)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::CameraLens> CameraLensPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::CameraLens const> ConstCameraLensPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_camera_5flens_2eproto__INCLUDED
