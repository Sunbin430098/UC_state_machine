// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: quaternion.proto

#ifndef PROTOBUF_quaternion_2eproto__INCLUDED
#define PROTOBUF_quaternion_2eproto__INCLUDED

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
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_quaternion_2eproto();
void protobuf_AssignDesc_quaternion_2eproto();
void protobuf_ShutdownFile_quaternion_2eproto();

class Quaternion;

// ===================================================================

class GZ_MSGS_VISIBLE Quaternion : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Quaternion) */ {
 public:
  Quaternion();
  virtual ~Quaternion();

  Quaternion(const Quaternion& from);

  inline Quaternion& operator=(const Quaternion& from) {
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
  static const Quaternion& default_instance();

  void Swap(Quaternion* other);

  // implements Message ----------------------------------------------

  inline Quaternion* New() const { return New(NULL); }

  Quaternion* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Quaternion& from);
  void MergeFrom(const Quaternion& from);
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
  void InternalSwap(Quaternion* other);
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

  // required double x = 2;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 2;
  double x() const;
  void set_x(double value);

  // required double y = 3;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 3;
  double y() const;
  void set_y(double value);

  // required double z = 4;
  bool has_z() const;
  void clear_z();
  static const int kZFieldNumber = 4;
  double z() const;
  void set_z(double value);

  // required double w = 5;
  bool has_w() const;
  void clear_w();
  static const int kWFieldNumber = 5;
  double w() const;
  void set_w(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Quaternion)
 private:
  inline void set_has_x();
  inline void clear_has_x();
  inline void set_has_y();
  inline void clear_has_y();
  inline void set_has_z();
  inline void clear_has_z();
  inline void set_has_w();
  inline void clear_has_w();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double x_;
  double y_;
  double z_;
  double w_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_quaternion_2eproto();
  friend void protobuf_AssignDesc_quaternion_2eproto();
  friend void protobuf_ShutdownFile_quaternion_2eproto();

  void InitAsDefaultInstance();
  static Quaternion* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Quaternion

// required double x = 2;
inline bool Quaternion::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Quaternion::set_has_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Quaternion::clear_has_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Quaternion::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline double Quaternion::x() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Quaternion.x)
  return x_;
}
inline void Quaternion::set_x(double value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Quaternion.x)
}

// required double y = 3;
inline bool Quaternion::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Quaternion::set_has_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Quaternion::clear_has_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Quaternion::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline double Quaternion::y() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Quaternion.y)
  return y_;
}
inline void Quaternion::set_y(double value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Quaternion.y)
}

// required double z = 4;
inline bool Quaternion::has_z() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Quaternion::set_has_z() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Quaternion::clear_has_z() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Quaternion::clear_z() {
  z_ = 0;
  clear_has_z();
}
inline double Quaternion::z() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Quaternion.z)
  return z_;
}
inline void Quaternion::set_z(double value) {
  set_has_z();
  z_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Quaternion.z)
}

// required double w = 5;
inline bool Quaternion::has_w() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Quaternion::set_has_w() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Quaternion::clear_has_w() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Quaternion::clear_w() {
  w_ = 0;
  clear_has_w();
}
inline double Quaternion::w() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Quaternion.w)
  return w_;
}
inline void Quaternion::set_w(double value) {
  set_has_w();
  w_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Quaternion.w)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Quaternion> QuaternionPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Quaternion const> ConstQuaternionPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_quaternion_2eproto__INCLUDED
