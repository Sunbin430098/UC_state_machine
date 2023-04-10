// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: inertial.proto

#ifndef PROTOBUF_inertial_2eproto__INCLUDED
#define PROTOBUF_inertial_2eproto__INCLUDED

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
#include "pose.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_inertial_2eproto();
void protobuf_AssignDesc_inertial_2eproto();
void protobuf_ShutdownFile_inertial_2eproto();

class Inertial;

// ===================================================================

class GZ_MSGS_VISIBLE Inertial : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Inertial) */ {
 public:
  Inertial();
  virtual ~Inertial();

  Inertial(const Inertial& from);

  inline Inertial& operator=(const Inertial& from) {
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
  static const Inertial& default_instance();

  void Swap(Inertial* other);

  // implements Message ----------------------------------------------

  inline Inertial* New() const { return New(NULL); }

  Inertial* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Inertial& from);
  void MergeFrom(const Inertial& from);
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
  void InternalSwap(Inertial* other);
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

  // optional double mass = 1;
  bool has_mass() const;
  void clear_mass();
  static const int kMassFieldNumber = 1;
  double mass() const;
  void set_mass(double value);

  // optional .gazebo.msgs.Pose pose = 2;
  bool has_pose() const;
  void clear_pose();
  static const int kPoseFieldNumber = 2;
  const ::gazebo::msgs::Pose& pose() const;
  ::gazebo::msgs::Pose* mutable_pose();
  ::gazebo::msgs::Pose* release_pose();
  void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // optional double ixx = 3;
  bool has_ixx() const;
  void clear_ixx();
  static const int kIxxFieldNumber = 3;
  double ixx() const;
  void set_ixx(double value);

  // optional double ixy = 4;
  bool has_ixy() const;
  void clear_ixy();
  static const int kIxyFieldNumber = 4;
  double ixy() const;
  void set_ixy(double value);

  // optional double ixz = 5;
  bool has_ixz() const;
  void clear_ixz();
  static const int kIxzFieldNumber = 5;
  double ixz() const;
  void set_ixz(double value);

  // optional double iyy = 6;
  bool has_iyy() const;
  void clear_iyy();
  static const int kIyyFieldNumber = 6;
  double iyy() const;
  void set_iyy(double value);

  // optional double iyz = 7;
  bool has_iyz() const;
  void clear_iyz();
  static const int kIyzFieldNumber = 7;
  double iyz() const;
  void set_iyz(double value);

  // optional double izz = 8;
  bool has_izz() const;
  void clear_izz();
  static const int kIzzFieldNumber = 8;
  double izz() const;
  void set_izz(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Inertial)
 private:
  inline void set_has_mass();
  inline void clear_has_mass();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_ixx();
  inline void clear_has_ixx();
  inline void set_has_ixy();
  inline void clear_has_ixy();
  inline void set_has_ixz();
  inline void clear_has_ixz();
  inline void set_has_iyy();
  inline void clear_has_iyy();
  inline void set_has_iyz();
  inline void clear_has_iyz();
  inline void set_has_izz();
  inline void clear_has_izz();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double mass_;
  ::gazebo::msgs::Pose* pose_;
  double ixx_;
  double ixy_;
  double ixz_;
  double iyy_;
  double iyz_;
  double izz_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_inertial_2eproto();
  friend void protobuf_AssignDesc_inertial_2eproto();
  friend void protobuf_ShutdownFile_inertial_2eproto();

  void InitAsDefaultInstance();
  static Inertial* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Inertial

// optional double mass = 1;
inline bool Inertial::has_mass() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Inertial::set_has_mass() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Inertial::clear_has_mass() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Inertial::clear_mass() {
  mass_ = 0;
  clear_has_mass();
}
inline double Inertial::mass() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.mass)
  return mass_;
}
inline void Inertial::set_mass(double value) {
  set_has_mass();
  mass_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.mass)
}

// optional .gazebo.msgs.Pose pose = 2;
inline bool Inertial::has_pose() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Inertial::set_has_pose() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Inertial::clear_has_pose() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Inertial::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& Inertial::pose() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.pose)
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* Inertial::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) {
    pose_ = new ::gazebo::msgs::Pose;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Inertial.pose)
  return pose_;
}
inline ::gazebo::msgs::Pose* Inertial::release_pose() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Inertial.pose)
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void Inertial::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Inertial.pose)
}

// optional double ixx = 3;
inline bool Inertial::has_ixx() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Inertial::set_has_ixx() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Inertial::clear_has_ixx() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Inertial::clear_ixx() {
  ixx_ = 0;
  clear_has_ixx();
}
inline double Inertial::ixx() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.ixx)
  return ixx_;
}
inline void Inertial::set_ixx(double value) {
  set_has_ixx();
  ixx_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.ixx)
}

// optional double ixy = 4;
inline bool Inertial::has_ixy() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Inertial::set_has_ixy() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Inertial::clear_has_ixy() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Inertial::clear_ixy() {
  ixy_ = 0;
  clear_has_ixy();
}
inline double Inertial::ixy() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.ixy)
  return ixy_;
}
inline void Inertial::set_ixy(double value) {
  set_has_ixy();
  ixy_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.ixy)
}

// optional double ixz = 5;
inline bool Inertial::has_ixz() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Inertial::set_has_ixz() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Inertial::clear_has_ixz() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Inertial::clear_ixz() {
  ixz_ = 0;
  clear_has_ixz();
}
inline double Inertial::ixz() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.ixz)
  return ixz_;
}
inline void Inertial::set_ixz(double value) {
  set_has_ixz();
  ixz_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.ixz)
}

// optional double iyy = 6;
inline bool Inertial::has_iyy() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Inertial::set_has_iyy() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Inertial::clear_has_iyy() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Inertial::clear_iyy() {
  iyy_ = 0;
  clear_has_iyy();
}
inline double Inertial::iyy() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.iyy)
  return iyy_;
}
inline void Inertial::set_iyy(double value) {
  set_has_iyy();
  iyy_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.iyy)
}

// optional double iyz = 7;
inline bool Inertial::has_iyz() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Inertial::set_has_iyz() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Inertial::clear_has_iyz() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Inertial::clear_iyz() {
  iyz_ = 0;
  clear_has_iyz();
}
inline double Inertial::iyz() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.iyz)
  return iyz_;
}
inline void Inertial::set_iyz(double value) {
  set_has_iyz();
  iyz_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.iyz)
}

// optional double izz = 8;
inline bool Inertial::has_izz() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Inertial::set_has_izz() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Inertial::clear_has_izz() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Inertial::clear_izz() {
  izz_ = 0;
  clear_has_izz();
}
inline double Inertial::izz() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Inertial.izz)
  return izz_;
}
inline void Inertial::set_izz(double value) {
  set_has_izz();
  izz_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Inertial.izz)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Inertial> InertialPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Inertial const> ConstInertialPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_inertial_2eproto__INCLUDED