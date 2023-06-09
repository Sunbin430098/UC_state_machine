// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: friction.proto

#ifndef PROTOBUF_friction_2eproto__INCLUDED
#define PROTOBUF_friction_2eproto__INCLUDED

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
#include "vector3d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_friction_2eproto();
void protobuf_AssignDesc_friction_2eproto();
void protobuf_ShutdownFile_friction_2eproto();

class Friction;
class Friction_Torsional;
class Friction_Torsional_ODE;

// ===================================================================

class GZ_MSGS_VISIBLE Friction_Torsional_ODE : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Friction.Torsional.ODE) */ {
 public:
  Friction_Torsional_ODE();
  virtual ~Friction_Torsional_ODE();

  Friction_Torsional_ODE(const Friction_Torsional_ODE& from);

  inline Friction_Torsional_ODE& operator=(const Friction_Torsional_ODE& from) {
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
  static const Friction_Torsional_ODE& default_instance();

  void Swap(Friction_Torsional_ODE* other);

  // implements Message ----------------------------------------------

  inline Friction_Torsional_ODE* New() const { return New(NULL); }

  Friction_Torsional_ODE* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Friction_Torsional_ODE& from);
  void MergeFrom(const Friction_Torsional_ODE& from);
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
  void InternalSwap(Friction_Torsional_ODE* other);
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

  // optional double slip = 1;
  bool has_slip() const;
  void clear_slip();
  static const int kSlipFieldNumber = 1;
  double slip() const;
  void set_slip(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Friction.Torsional.ODE)
 private:
  inline void set_has_slip();
  inline void clear_has_slip();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double slip_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_friction_2eproto();
  friend void protobuf_AssignDesc_friction_2eproto();
  friend void protobuf_ShutdownFile_friction_2eproto();

  void InitAsDefaultInstance();
  static Friction_Torsional_ODE* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE Friction_Torsional : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Friction.Torsional) */ {
 public:
  Friction_Torsional();
  virtual ~Friction_Torsional();

  Friction_Torsional(const Friction_Torsional& from);

  inline Friction_Torsional& operator=(const Friction_Torsional& from) {
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
  static const Friction_Torsional& default_instance();

  void Swap(Friction_Torsional* other);

  // implements Message ----------------------------------------------

  inline Friction_Torsional* New() const { return New(NULL); }

  Friction_Torsional* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Friction_Torsional& from);
  void MergeFrom(const Friction_Torsional& from);
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
  void InternalSwap(Friction_Torsional* other);
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

  typedef Friction_Torsional_ODE ODE;

  // accessors -------------------------------------------------------

  // optional double coefficient = 1;
  bool has_coefficient() const;
  void clear_coefficient();
  static const int kCoefficientFieldNumber = 1;
  double coefficient() const;
  void set_coefficient(double value);

  // optional bool use_patch_radius = 2;
  bool has_use_patch_radius() const;
  void clear_use_patch_radius();
  static const int kUsePatchRadiusFieldNumber = 2;
  bool use_patch_radius() const;
  void set_use_patch_radius(bool value);

  // optional double patch_radius = 3;
  bool has_patch_radius() const;
  void clear_patch_radius();
  static const int kPatchRadiusFieldNumber = 3;
  double patch_radius() const;
  void set_patch_radius(double value);

  // optional double surface_radius = 4;
  bool has_surface_radius() const;
  void clear_surface_radius();
  static const int kSurfaceRadiusFieldNumber = 4;
  double surface_radius() const;
  void set_surface_radius(double value);

  // optional .gazebo.msgs.Friction.Torsional.ODE ode = 5;
  bool has_ode() const;
  void clear_ode();
  static const int kOdeFieldNumber = 5;
  const ::gazebo::msgs::Friction_Torsional_ODE& ode() const;
  ::gazebo::msgs::Friction_Torsional_ODE* mutable_ode();
  ::gazebo::msgs::Friction_Torsional_ODE* release_ode();
  void set_allocated_ode(::gazebo::msgs::Friction_Torsional_ODE* ode);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Friction.Torsional)
 private:
  inline void set_has_coefficient();
  inline void clear_has_coefficient();
  inline void set_has_use_patch_radius();
  inline void clear_has_use_patch_radius();
  inline void set_has_patch_radius();
  inline void clear_has_patch_radius();
  inline void set_has_surface_radius();
  inline void clear_has_surface_radius();
  inline void set_has_ode();
  inline void clear_has_ode();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double coefficient_;
  double patch_radius_;
  double surface_radius_;
  ::gazebo::msgs::Friction_Torsional_ODE* ode_;
  bool use_patch_radius_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_friction_2eproto();
  friend void protobuf_AssignDesc_friction_2eproto();
  friend void protobuf_ShutdownFile_friction_2eproto();

  void InitAsDefaultInstance();
  static Friction_Torsional* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE Friction : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Friction) */ {
 public:
  Friction();
  virtual ~Friction();

  Friction(const Friction& from);

  inline Friction& operator=(const Friction& from) {
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
  static const Friction& default_instance();

  void Swap(Friction* other);

  // implements Message ----------------------------------------------

  inline Friction* New() const { return New(NULL); }

  Friction* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Friction& from);
  void MergeFrom(const Friction& from);
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
  void InternalSwap(Friction* other);
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

  typedef Friction_Torsional Torsional;

  // accessors -------------------------------------------------------

  // optional double mu = 1;
  bool has_mu() const;
  void clear_mu();
  static const int kMuFieldNumber = 1;
  double mu() const;
  void set_mu(double value);

  // optional double mu2 = 2;
  bool has_mu2() const;
  void clear_mu2();
  static const int kMu2FieldNumber = 2;
  double mu2() const;
  void set_mu2(double value);

  // optional .gazebo.msgs.Vector3d fdir1 = 3;
  bool has_fdir1() const;
  void clear_fdir1();
  static const int kFdir1FieldNumber = 3;
  const ::gazebo::msgs::Vector3d& fdir1() const;
  ::gazebo::msgs::Vector3d* mutable_fdir1();
  ::gazebo::msgs::Vector3d* release_fdir1();
  void set_allocated_fdir1(::gazebo::msgs::Vector3d* fdir1);

  // optional double slip1 = 4;
  bool has_slip1() const;
  void clear_slip1();
  static const int kSlip1FieldNumber = 4;
  double slip1() const;
  void set_slip1(double value);

  // optional double slip2 = 5;
  bool has_slip2() const;
  void clear_slip2();
  static const int kSlip2FieldNumber = 5;
  double slip2() const;
  void set_slip2(double value);

  // optional .gazebo.msgs.Friction.Torsional torsional = 6;
  bool has_torsional() const;
  void clear_torsional();
  static const int kTorsionalFieldNumber = 6;
  const ::gazebo::msgs::Friction_Torsional& torsional() const;
  ::gazebo::msgs::Friction_Torsional* mutable_torsional();
  ::gazebo::msgs::Friction_Torsional* release_torsional();
  void set_allocated_torsional(::gazebo::msgs::Friction_Torsional* torsional);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Friction)
 private:
  inline void set_has_mu();
  inline void clear_has_mu();
  inline void set_has_mu2();
  inline void clear_has_mu2();
  inline void set_has_fdir1();
  inline void clear_has_fdir1();
  inline void set_has_slip1();
  inline void clear_has_slip1();
  inline void set_has_slip2();
  inline void clear_has_slip2();
  inline void set_has_torsional();
  inline void clear_has_torsional();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double mu_;
  double mu2_;
  ::gazebo::msgs::Vector3d* fdir1_;
  double slip1_;
  double slip2_;
  ::gazebo::msgs::Friction_Torsional* torsional_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_friction_2eproto();
  friend void protobuf_AssignDesc_friction_2eproto();
  friend void protobuf_ShutdownFile_friction_2eproto();

  void InitAsDefaultInstance();
  static Friction* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Friction_Torsional_ODE

// optional double slip = 1;
inline bool Friction_Torsional_ODE::has_slip() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Friction_Torsional_ODE::set_has_slip() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Friction_Torsional_ODE::clear_has_slip() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Friction_Torsional_ODE::clear_slip() {
  slip_ = 0;
  clear_has_slip();
}
inline double Friction_Torsional_ODE::slip() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.Torsional.ODE.slip)
  return slip_;
}
inline void Friction_Torsional_ODE::set_slip(double value) {
  set_has_slip();
  slip_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.Torsional.ODE.slip)
}

// -------------------------------------------------------------------

// Friction_Torsional

// optional double coefficient = 1;
inline bool Friction_Torsional::has_coefficient() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Friction_Torsional::set_has_coefficient() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Friction_Torsional::clear_has_coefficient() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Friction_Torsional::clear_coefficient() {
  coefficient_ = 0;
  clear_has_coefficient();
}
inline double Friction_Torsional::coefficient() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.Torsional.coefficient)
  return coefficient_;
}
inline void Friction_Torsional::set_coefficient(double value) {
  set_has_coefficient();
  coefficient_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.Torsional.coefficient)
}

// optional bool use_patch_radius = 2;
inline bool Friction_Torsional::has_use_patch_radius() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Friction_Torsional::set_has_use_patch_radius() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Friction_Torsional::clear_has_use_patch_radius() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Friction_Torsional::clear_use_patch_radius() {
  use_patch_radius_ = false;
  clear_has_use_patch_radius();
}
inline bool Friction_Torsional::use_patch_radius() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.Torsional.use_patch_radius)
  return use_patch_radius_;
}
inline void Friction_Torsional::set_use_patch_radius(bool value) {
  set_has_use_patch_radius();
  use_patch_radius_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.Torsional.use_patch_radius)
}

// optional double patch_radius = 3;
inline bool Friction_Torsional::has_patch_radius() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Friction_Torsional::set_has_patch_radius() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Friction_Torsional::clear_has_patch_radius() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Friction_Torsional::clear_patch_radius() {
  patch_radius_ = 0;
  clear_has_patch_radius();
}
inline double Friction_Torsional::patch_radius() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.Torsional.patch_radius)
  return patch_radius_;
}
inline void Friction_Torsional::set_patch_radius(double value) {
  set_has_patch_radius();
  patch_radius_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.Torsional.patch_radius)
}

// optional double surface_radius = 4;
inline bool Friction_Torsional::has_surface_radius() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Friction_Torsional::set_has_surface_radius() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Friction_Torsional::clear_has_surface_radius() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Friction_Torsional::clear_surface_radius() {
  surface_radius_ = 0;
  clear_has_surface_radius();
}
inline double Friction_Torsional::surface_radius() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.Torsional.surface_radius)
  return surface_radius_;
}
inline void Friction_Torsional::set_surface_radius(double value) {
  set_has_surface_radius();
  surface_radius_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.Torsional.surface_radius)
}

// optional .gazebo.msgs.Friction.Torsional.ODE ode = 5;
inline bool Friction_Torsional::has_ode() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Friction_Torsional::set_has_ode() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Friction_Torsional::clear_has_ode() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Friction_Torsional::clear_ode() {
  if (ode_ != NULL) ode_->::gazebo::msgs::Friction_Torsional_ODE::Clear();
  clear_has_ode();
}
inline const ::gazebo::msgs::Friction_Torsional_ODE& Friction_Torsional::ode() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.Torsional.ode)
  return ode_ != NULL ? *ode_ : *default_instance_->ode_;
}
inline ::gazebo::msgs::Friction_Torsional_ODE* Friction_Torsional::mutable_ode() {
  set_has_ode();
  if (ode_ == NULL) {
    ode_ = new ::gazebo::msgs::Friction_Torsional_ODE;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Friction.Torsional.ode)
  return ode_;
}
inline ::gazebo::msgs::Friction_Torsional_ODE* Friction_Torsional::release_ode() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Friction.Torsional.ode)
  clear_has_ode();
  ::gazebo::msgs::Friction_Torsional_ODE* temp = ode_;
  ode_ = NULL;
  return temp;
}
inline void Friction_Torsional::set_allocated_ode(::gazebo::msgs::Friction_Torsional_ODE* ode) {
  delete ode_;
  ode_ = ode;
  if (ode) {
    set_has_ode();
  } else {
    clear_has_ode();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Friction.Torsional.ode)
}

// -------------------------------------------------------------------

// Friction

// optional double mu = 1;
inline bool Friction::has_mu() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Friction::set_has_mu() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Friction::clear_has_mu() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Friction::clear_mu() {
  mu_ = 0;
  clear_has_mu();
}
inline double Friction::mu() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.mu)
  return mu_;
}
inline void Friction::set_mu(double value) {
  set_has_mu();
  mu_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.mu)
}

// optional double mu2 = 2;
inline bool Friction::has_mu2() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Friction::set_has_mu2() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Friction::clear_has_mu2() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Friction::clear_mu2() {
  mu2_ = 0;
  clear_has_mu2();
}
inline double Friction::mu2() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.mu2)
  return mu2_;
}
inline void Friction::set_mu2(double value) {
  set_has_mu2();
  mu2_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.mu2)
}

// optional .gazebo.msgs.Vector3d fdir1 = 3;
inline bool Friction::has_fdir1() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Friction::set_has_fdir1() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Friction::clear_has_fdir1() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Friction::clear_fdir1() {
  if (fdir1_ != NULL) fdir1_->::gazebo::msgs::Vector3d::Clear();
  clear_has_fdir1();
}
inline const ::gazebo::msgs::Vector3d& Friction::fdir1() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.fdir1)
  return fdir1_ != NULL ? *fdir1_ : *default_instance_->fdir1_;
}
inline ::gazebo::msgs::Vector3d* Friction::mutable_fdir1() {
  set_has_fdir1();
  if (fdir1_ == NULL) {
    fdir1_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Friction.fdir1)
  return fdir1_;
}
inline ::gazebo::msgs::Vector3d* Friction::release_fdir1() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Friction.fdir1)
  clear_has_fdir1();
  ::gazebo::msgs::Vector3d* temp = fdir1_;
  fdir1_ = NULL;
  return temp;
}
inline void Friction::set_allocated_fdir1(::gazebo::msgs::Vector3d* fdir1) {
  delete fdir1_;
  fdir1_ = fdir1;
  if (fdir1) {
    set_has_fdir1();
  } else {
    clear_has_fdir1();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Friction.fdir1)
}

// optional double slip1 = 4;
inline bool Friction::has_slip1() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Friction::set_has_slip1() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Friction::clear_has_slip1() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Friction::clear_slip1() {
  slip1_ = 0;
  clear_has_slip1();
}
inline double Friction::slip1() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.slip1)
  return slip1_;
}
inline void Friction::set_slip1(double value) {
  set_has_slip1();
  slip1_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.slip1)
}

// optional double slip2 = 5;
inline bool Friction::has_slip2() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Friction::set_has_slip2() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Friction::clear_has_slip2() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Friction::clear_slip2() {
  slip2_ = 0;
  clear_has_slip2();
}
inline double Friction::slip2() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.slip2)
  return slip2_;
}
inline void Friction::set_slip2(double value) {
  set_has_slip2();
  slip2_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Friction.slip2)
}

// optional .gazebo.msgs.Friction.Torsional torsional = 6;
inline bool Friction::has_torsional() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Friction::set_has_torsional() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Friction::clear_has_torsional() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Friction::clear_torsional() {
  if (torsional_ != NULL) torsional_->::gazebo::msgs::Friction_Torsional::Clear();
  clear_has_torsional();
}
inline const ::gazebo::msgs::Friction_Torsional& Friction::torsional() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Friction.torsional)
  return torsional_ != NULL ? *torsional_ : *default_instance_->torsional_;
}
inline ::gazebo::msgs::Friction_Torsional* Friction::mutable_torsional() {
  set_has_torsional();
  if (torsional_ == NULL) {
    torsional_ = new ::gazebo::msgs::Friction_Torsional;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Friction.torsional)
  return torsional_;
}
inline ::gazebo::msgs::Friction_Torsional* Friction::release_torsional() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Friction.torsional)
  clear_has_torsional();
  ::gazebo::msgs::Friction_Torsional* temp = torsional_;
  torsional_ = NULL;
  return temp;
}
inline void Friction::set_allocated_torsional(::gazebo::msgs::Friction_Torsional* torsional) {
  delete torsional_;
  torsional_ = torsional;
  if (torsional) {
    set_has_torsional();
  } else {
    clear_has_torsional();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Friction.torsional)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------


typedef boost::shared_ptr<gazebo::msgs::Friction> FrictionPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Friction const> ConstFrictionPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_friction_2eproto__INCLUDED
