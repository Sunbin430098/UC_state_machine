// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: imu_sensor.proto

#ifndef PROTOBUF_imu_5fsensor_2eproto__INCLUDED
#define PROTOBUF_imu_5fsensor_2eproto__INCLUDED

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
#include "sensor_noise.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_imu_5fsensor_2eproto();
void protobuf_AssignDesc_imu_5fsensor_2eproto();
void protobuf_ShutdownFile_imu_5fsensor_2eproto();

class IMUSensor;
class IMUSensor_AngularVelocity;
class IMUSensor_LinearAcceleration;

// ===================================================================

class GZ_MSGS_VISIBLE IMUSensor_AngularVelocity : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.IMUSensor.AngularVelocity) */ {
 public:
  IMUSensor_AngularVelocity();
  virtual ~IMUSensor_AngularVelocity();

  IMUSensor_AngularVelocity(const IMUSensor_AngularVelocity& from);

  inline IMUSensor_AngularVelocity& operator=(const IMUSensor_AngularVelocity& from) {
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
  static const IMUSensor_AngularVelocity& default_instance();

  void Swap(IMUSensor_AngularVelocity* other);

  // implements Message ----------------------------------------------

  inline IMUSensor_AngularVelocity* New() const { return New(NULL); }

  IMUSensor_AngularVelocity* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const IMUSensor_AngularVelocity& from);
  void MergeFrom(const IMUSensor_AngularVelocity& from);
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
  void InternalSwap(IMUSensor_AngularVelocity* other);
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

  // optional .gazebo.msgs.SensorNoise x_noise = 1;
  bool has_x_noise() const;
  void clear_x_noise();
  static const int kXNoiseFieldNumber = 1;
  const ::gazebo::msgs::SensorNoise& x_noise() const;
  ::gazebo::msgs::SensorNoise* mutable_x_noise();
  ::gazebo::msgs::SensorNoise* release_x_noise();
  void set_allocated_x_noise(::gazebo::msgs::SensorNoise* x_noise);

  // optional .gazebo.msgs.SensorNoise y_noise = 2;
  bool has_y_noise() const;
  void clear_y_noise();
  static const int kYNoiseFieldNumber = 2;
  const ::gazebo::msgs::SensorNoise& y_noise() const;
  ::gazebo::msgs::SensorNoise* mutable_y_noise();
  ::gazebo::msgs::SensorNoise* release_y_noise();
  void set_allocated_y_noise(::gazebo::msgs::SensorNoise* y_noise);

  // optional .gazebo.msgs.SensorNoise z_noise = 3;
  bool has_z_noise() const;
  void clear_z_noise();
  static const int kZNoiseFieldNumber = 3;
  const ::gazebo::msgs::SensorNoise& z_noise() const;
  ::gazebo::msgs::SensorNoise* mutable_z_noise();
  ::gazebo::msgs::SensorNoise* release_z_noise();
  void set_allocated_z_noise(::gazebo::msgs::SensorNoise* z_noise);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.IMUSensor.AngularVelocity)
 private:
  inline void set_has_x_noise();
  inline void clear_has_x_noise();
  inline void set_has_y_noise();
  inline void clear_has_y_noise();
  inline void set_has_z_noise();
  inline void clear_has_z_noise();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::SensorNoise* x_noise_;
  ::gazebo::msgs::SensorNoise* y_noise_;
  ::gazebo::msgs::SensorNoise* z_noise_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_imu_5fsensor_2eproto();
  friend void protobuf_AssignDesc_imu_5fsensor_2eproto();
  friend void protobuf_ShutdownFile_imu_5fsensor_2eproto();

  void InitAsDefaultInstance();
  static IMUSensor_AngularVelocity* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE IMUSensor_LinearAcceleration : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.IMUSensor.LinearAcceleration) */ {
 public:
  IMUSensor_LinearAcceleration();
  virtual ~IMUSensor_LinearAcceleration();

  IMUSensor_LinearAcceleration(const IMUSensor_LinearAcceleration& from);

  inline IMUSensor_LinearAcceleration& operator=(const IMUSensor_LinearAcceleration& from) {
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
  static const IMUSensor_LinearAcceleration& default_instance();

  void Swap(IMUSensor_LinearAcceleration* other);

  // implements Message ----------------------------------------------

  inline IMUSensor_LinearAcceleration* New() const { return New(NULL); }

  IMUSensor_LinearAcceleration* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const IMUSensor_LinearAcceleration& from);
  void MergeFrom(const IMUSensor_LinearAcceleration& from);
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
  void InternalSwap(IMUSensor_LinearAcceleration* other);
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

  // optional .gazebo.msgs.SensorNoise x_noise = 1;
  bool has_x_noise() const;
  void clear_x_noise();
  static const int kXNoiseFieldNumber = 1;
  const ::gazebo::msgs::SensorNoise& x_noise() const;
  ::gazebo::msgs::SensorNoise* mutable_x_noise();
  ::gazebo::msgs::SensorNoise* release_x_noise();
  void set_allocated_x_noise(::gazebo::msgs::SensorNoise* x_noise);

  // optional .gazebo.msgs.SensorNoise y_noise = 2;
  bool has_y_noise() const;
  void clear_y_noise();
  static const int kYNoiseFieldNumber = 2;
  const ::gazebo::msgs::SensorNoise& y_noise() const;
  ::gazebo::msgs::SensorNoise* mutable_y_noise();
  ::gazebo::msgs::SensorNoise* release_y_noise();
  void set_allocated_y_noise(::gazebo::msgs::SensorNoise* y_noise);

  // optional .gazebo.msgs.SensorNoise z_noise = 3;
  bool has_z_noise() const;
  void clear_z_noise();
  static const int kZNoiseFieldNumber = 3;
  const ::gazebo::msgs::SensorNoise& z_noise() const;
  ::gazebo::msgs::SensorNoise* mutable_z_noise();
  ::gazebo::msgs::SensorNoise* release_z_noise();
  void set_allocated_z_noise(::gazebo::msgs::SensorNoise* z_noise);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.IMUSensor.LinearAcceleration)
 private:
  inline void set_has_x_noise();
  inline void clear_has_x_noise();
  inline void set_has_y_noise();
  inline void clear_has_y_noise();
  inline void set_has_z_noise();
  inline void clear_has_z_noise();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::SensorNoise* x_noise_;
  ::gazebo::msgs::SensorNoise* y_noise_;
  ::gazebo::msgs::SensorNoise* z_noise_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_imu_5fsensor_2eproto();
  friend void protobuf_AssignDesc_imu_5fsensor_2eproto();
  friend void protobuf_ShutdownFile_imu_5fsensor_2eproto();

  void InitAsDefaultInstance();
  static IMUSensor_LinearAcceleration* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE IMUSensor : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.IMUSensor) */ {
 public:
  IMUSensor();
  virtual ~IMUSensor();

  IMUSensor(const IMUSensor& from);

  inline IMUSensor& operator=(const IMUSensor& from) {
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
  static const IMUSensor& default_instance();

  void Swap(IMUSensor* other);

  // implements Message ----------------------------------------------

  inline IMUSensor* New() const { return New(NULL); }

  IMUSensor* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const IMUSensor& from);
  void MergeFrom(const IMUSensor& from);
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
  void InternalSwap(IMUSensor* other);
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

  typedef IMUSensor_AngularVelocity AngularVelocity;
  typedef IMUSensor_LinearAcceleration LinearAcceleration;

  // accessors -------------------------------------------------------

  // optional .gazebo.msgs.IMUSensor.AngularVelocity angular_velocity = 1;
  bool has_angular_velocity() const;
  void clear_angular_velocity();
  static const int kAngularVelocityFieldNumber = 1;
  const ::gazebo::msgs::IMUSensor_AngularVelocity& angular_velocity() const;
  ::gazebo::msgs::IMUSensor_AngularVelocity* mutable_angular_velocity();
  ::gazebo::msgs::IMUSensor_AngularVelocity* release_angular_velocity();
  void set_allocated_angular_velocity(::gazebo::msgs::IMUSensor_AngularVelocity* angular_velocity);

  // optional .gazebo.msgs.IMUSensor.LinearAcceleration linear_acceleration = 2;
  bool has_linear_acceleration() const;
  void clear_linear_acceleration();
  static const int kLinearAccelerationFieldNumber = 2;
  const ::gazebo::msgs::IMUSensor_LinearAcceleration& linear_acceleration() const;
  ::gazebo::msgs::IMUSensor_LinearAcceleration* mutable_linear_acceleration();
  ::gazebo::msgs::IMUSensor_LinearAcceleration* release_linear_acceleration();
  void set_allocated_linear_acceleration(::gazebo::msgs::IMUSensor_LinearAcceleration* linear_acceleration);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.IMUSensor)
 private:
  inline void set_has_angular_velocity();
  inline void clear_has_angular_velocity();
  inline void set_has_linear_acceleration();
  inline void clear_has_linear_acceleration();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::IMUSensor_AngularVelocity* angular_velocity_;
  ::gazebo::msgs::IMUSensor_LinearAcceleration* linear_acceleration_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_imu_5fsensor_2eproto();
  friend void protobuf_AssignDesc_imu_5fsensor_2eproto();
  friend void protobuf_ShutdownFile_imu_5fsensor_2eproto();

  void InitAsDefaultInstance();
  static IMUSensor* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// IMUSensor_AngularVelocity

// optional .gazebo.msgs.SensorNoise x_noise = 1;
inline bool IMUSensor_AngularVelocity::has_x_noise() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void IMUSensor_AngularVelocity::set_has_x_noise() {
  _has_bits_[0] |= 0x00000001u;
}
inline void IMUSensor_AngularVelocity::clear_has_x_noise() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void IMUSensor_AngularVelocity::clear_x_noise() {
  if (x_noise_ != NULL) x_noise_->::gazebo::msgs::SensorNoise::Clear();
  clear_has_x_noise();
}
inline const ::gazebo::msgs::SensorNoise& IMUSensor_AngularVelocity::x_noise() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.AngularVelocity.x_noise)
  return x_noise_ != NULL ? *x_noise_ : *default_instance_->x_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_AngularVelocity::mutable_x_noise() {
  set_has_x_noise();
  if (x_noise_ == NULL) {
    x_noise_ = new ::gazebo::msgs::SensorNoise;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.AngularVelocity.x_noise)
  return x_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_AngularVelocity::release_x_noise() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.AngularVelocity.x_noise)
  clear_has_x_noise();
  ::gazebo::msgs::SensorNoise* temp = x_noise_;
  x_noise_ = NULL;
  return temp;
}
inline void IMUSensor_AngularVelocity::set_allocated_x_noise(::gazebo::msgs::SensorNoise* x_noise) {
  delete x_noise_;
  x_noise_ = x_noise;
  if (x_noise) {
    set_has_x_noise();
  } else {
    clear_has_x_noise();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.AngularVelocity.x_noise)
}

// optional .gazebo.msgs.SensorNoise y_noise = 2;
inline bool IMUSensor_AngularVelocity::has_y_noise() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void IMUSensor_AngularVelocity::set_has_y_noise() {
  _has_bits_[0] |= 0x00000002u;
}
inline void IMUSensor_AngularVelocity::clear_has_y_noise() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void IMUSensor_AngularVelocity::clear_y_noise() {
  if (y_noise_ != NULL) y_noise_->::gazebo::msgs::SensorNoise::Clear();
  clear_has_y_noise();
}
inline const ::gazebo::msgs::SensorNoise& IMUSensor_AngularVelocity::y_noise() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.AngularVelocity.y_noise)
  return y_noise_ != NULL ? *y_noise_ : *default_instance_->y_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_AngularVelocity::mutable_y_noise() {
  set_has_y_noise();
  if (y_noise_ == NULL) {
    y_noise_ = new ::gazebo::msgs::SensorNoise;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.AngularVelocity.y_noise)
  return y_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_AngularVelocity::release_y_noise() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.AngularVelocity.y_noise)
  clear_has_y_noise();
  ::gazebo::msgs::SensorNoise* temp = y_noise_;
  y_noise_ = NULL;
  return temp;
}
inline void IMUSensor_AngularVelocity::set_allocated_y_noise(::gazebo::msgs::SensorNoise* y_noise) {
  delete y_noise_;
  y_noise_ = y_noise;
  if (y_noise) {
    set_has_y_noise();
  } else {
    clear_has_y_noise();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.AngularVelocity.y_noise)
}

// optional .gazebo.msgs.SensorNoise z_noise = 3;
inline bool IMUSensor_AngularVelocity::has_z_noise() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void IMUSensor_AngularVelocity::set_has_z_noise() {
  _has_bits_[0] |= 0x00000004u;
}
inline void IMUSensor_AngularVelocity::clear_has_z_noise() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void IMUSensor_AngularVelocity::clear_z_noise() {
  if (z_noise_ != NULL) z_noise_->::gazebo::msgs::SensorNoise::Clear();
  clear_has_z_noise();
}
inline const ::gazebo::msgs::SensorNoise& IMUSensor_AngularVelocity::z_noise() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.AngularVelocity.z_noise)
  return z_noise_ != NULL ? *z_noise_ : *default_instance_->z_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_AngularVelocity::mutable_z_noise() {
  set_has_z_noise();
  if (z_noise_ == NULL) {
    z_noise_ = new ::gazebo::msgs::SensorNoise;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.AngularVelocity.z_noise)
  return z_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_AngularVelocity::release_z_noise() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.AngularVelocity.z_noise)
  clear_has_z_noise();
  ::gazebo::msgs::SensorNoise* temp = z_noise_;
  z_noise_ = NULL;
  return temp;
}
inline void IMUSensor_AngularVelocity::set_allocated_z_noise(::gazebo::msgs::SensorNoise* z_noise) {
  delete z_noise_;
  z_noise_ = z_noise;
  if (z_noise) {
    set_has_z_noise();
  } else {
    clear_has_z_noise();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.AngularVelocity.z_noise)
}

// -------------------------------------------------------------------

// IMUSensor_LinearAcceleration

// optional .gazebo.msgs.SensorNoise x_noise = 1;
inline bool IMUSensor_LinearAcceleration::has_x_noise() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void IMUSensor_LinearAcceleration::set_has_x_noise() {
  _has_bits_[0] |= 0x00000001u;
}
inline void IMUSensor_LinearAcceleration::clear_has_x_noise() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void IMUSensor_LinearAcceleration::clear_x_noise() {
  if (x_noise_ != NULL) x_noise_->::gazebo::msgs::SensorNoise::Clear();
  clear_has_x_noise();
}
inline const ::gazebo::msgs::SensorNoise& IMUSensor_LinearAcceleration::x_noise() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.LinearAcceleration.x_noise)
  return x_noise_ != NULL ? *x_noise_ : *default_instance_->x_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_LinearAcceleration::mutable_x_noise() {
  set_has_x_noise();
  if (x_noise_ == NULL) {
    x_noise_ = new ::gazebo::msgs::SensorNoise;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.LinearAcceleration.x_noise)
  return x_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_LinearAcceleration::release_x_noise() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.LinearAcceleration.x_noise)
  clear_has_x_noise();
  ::gazebo::msgs::SensorNoise* temp = x_noise_;
  x_noise_ = NULL;
  return temp;
}
inline void IMUSensor_LinearAcceleration::set_allocated_x_noise(::gazebo::msgs::SensorNoise* x_noise) {
  delete x_noise_;
  x_noise_ = x_noise;
  if (x_noise) {
    set_has_x_noise();
  } else {
    clear_has_x_noise();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.LinearAcceleration.x_noise)
}

// optional .gazebo.msgs.SensorNoise y_noise = 2;
inline bool IMUSensor_LinearAcceleration::has_y_noise() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void IMUSensor_LinearAcceleration::set_has_y_noise() {
  _has_bits_[0] |= 0x00000002u;
}
inline void IMUSensor_LinearAcceleration::clear_has_y_noise() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void IMUSensor_LinearAcceleration::clear_y_noise() {
  if (y_noise_ != NULL) y_noise_->::gazebo::msgs::SensorNoise::Clear();
  clear_has_y_noise();
}
inline const ::gazebo::msgs::SensorNoise& IMUSensor_LinearAcceleration::y_noise() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.LinearAcceleration.y_noise)
  return y_noise_ != NULL ? *y_noise_ : *default_instance_->y_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_LinearAcceleration::mutable_y_noise() {
  set_has_y_noise();
  if (y_noise_ == NULL) {
    y_noise_ = new ::gazebo::msgs::SensorNoise;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.LinearAcceleration.y_noise)
  return y_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_LinearAcceleration::release_y_noise() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.LinearAcceleration.y_noise)
  clear_has_y_noise();
  ::gazebo::msgs::SensorNoise* temp = y_noise_;
  y_noise_ = NULL;
  return temp;
}
inline void IMUSensor_LinearAcceleration::set_allocated_y_noise(::gazebo::msgs::SensorNoise* y_noise) {
  delete y_noise_;
  y_noise_ = y_noise;
  if (y_noise) {
    set_has_y_noise();
  } else {
    clear_has_y_noise();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.LinearAcceleration.y_noise)
}

// optional .gazebo.msgs.SensorNoise z_noise = 3;
inline bool IMUSensor_LinearAcceleration::has_z_noise() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void IMUSensor_LinearAcceleration::set_has_z_noise() {
  _has_bits_[0] |= 0x00000004u;
}
inline void IMUSensor_LinearAcceleration::clear_has_z_noise() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void IMUSensor_LinearAcceleration::clear_z_noise() {
  if (z_noise_ != NULL) z_noise_->::gazebo::msgs::SensorNoise::Clear();
  clear_has_z_noise();
}
inline const ::gazebo::msgs::SensorNoise& IMUSensor_LinearAcceleration::z_noise() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.LinearAcceleration.z_noise)
  return z_noise_ != NULL ? *z_noise_ : *default_instance_->z_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_LinearAcceleration::mutable_z_noise() {
  set_has_z_noise();
  if (z_noise_ == NULL) {
    z_noise_ = new ::gazebo::msgs::SensorNoise;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.LinearAcceleration.z_noise)
  return z_noise_;
}
inline ::gazebo::msgs::SensorNoise* IMUSensor_LinearAcceleration::release_z_noise() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.LinearAcceleration.z_noise)
  clear_has_z_noise();
  ::gazebo::msgs::SensorNoise* temp = z_noise_;
  z_noise_ = NULL;
  return temp;
}
inline void IMUSensor_LinearAcceleration::set_allocated_z_noise(::gazebo::msgs::SensorNoise* z_noise) {
  delete z_noise_;
  z_noise_ = z_noise;
  if (z_noise) {
    set_has_z_noise();
  } else {
    clear_has_z_noise();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.LinearAcceleration.z_noise)
}

// -------------------------------------------------------------------

// IMUSensor

// optional .gazebo.msgs.IMUSensor.AngularVelocity angular_velocity = 1;
inline bool IMUSensor::has_angular_velocity() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void IMUSensor::set_has_angular_velocity() {
  _has_bits_[0] |= 0x00000001u;
}
inline void IMUSensor::clear_has_angular_velocity() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void IMUSensor::clear_angular_velocity() {
  if (angular_velocity_ != NULL) angular_velocity_->::gazebo::msgs::IMUSensor_AngularVelocity::Clear();
  clear_has_angular_velocity();
}
inline const ::gazebo::msgs::IMUSensor_AngularVelocity& IMUSensor::angular_velocity() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.angular_velocity)
  return angular_velocity_ != NULL ? *angular_velocity_ : *default_instance_->angular_velocity_;
}
inline ::gazebo::msgs::IMUSensor_AngularVelocity* IMUSensor::mutable_angular_velocity() {
  set_has_angular_velocity();
  if (angular_velocity_ == NULL) {
    angular_velocity_ = new ::gazebo::msgs::IMUSensor_AngularVelocity;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.angular_velocity)
  return angular_velocity_;
}
inline ::gazebo::msgs::IMUSensor_AngularVelocity* IMUSensor::release_angular_velocity() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.angular_velocity)
  clear_has_angular_velocity();
  ::gazebo::msgs::IMUSensor_AngularVelocity* temp = angular_velocity_;
  angular_velocity_ = NULL;
  return temp;
}
inline void IMUSensor::set_allocated_angular_velocity(::gazebo::msgs::IMUSensor_AngularVelocity* angular_velocity) {
  delete angular_velocity_;
  angular_velocity_ = angular_velocity;
  if (angular_velocity) {
    set_has_angular_velocity();
  } else {
    clear_has_angular_velocity();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.angular_velocity)
}

// optional .gazebo.msgs.IMUSensor.LinearAcceleration linear_acceleration = 2;
inline bool IMUSensor::has_linear_acceleration() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void IMUSensor::set_has_linear_acceleration() {
  _has_bits_[0] |= 0x00000002u;
}
inline void IMUSensor::clear_has_linear_acceleration() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void IMUSensor::clear_linear_acceleration() {
  if (linear_acceleration_ != NULL) linear_acceleration_->::gazebo::msgs::IMUSensor_LinearAcceleration::Clear();
  clear_has_linear_acceleration();
}
inline const ::gazebo::msgs::IMUSensor_LinearAcceleration& IMUSensor::linear_acceleration() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.IMUSensor.linear_acceleration)
  return linear_acceleration_ != NULL ? *linear_acceleration_ : *default_instance_->linear_acceleration_;
}
inline ::gazebo::msgs::IMUSensor_LinearAcceleration* IMUSensor::mutable_linear_acceleration() {
  set_has_linear_acceleration();
  if (linear_acceleration_ == NULL) {
    linear_acceleration_ = new ::gazebo::msgs::IMUSensor_LinearAcceleration;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.IMUSensor.linear_acceleration)
  return linear_acceleration_;
}
inline ::gazebo::msgs::IMUSensor_LinearAcceleration* IMUSensor::release_linear_acceleration() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.IMUSensor.linear_acceleration)
  clear_has_linear_acceleration();
  ::gazebo::msgs::IMUSensor_LinearAcceleration* temp = linear_acceleration_;
  linear_acceleration_ = NULL;
  return temp;
}
inline void IMUSensor::set_allocated_linear_acceleration(::gazebo::msgs::IMUSensor_LinearAcceleration* linear_acceleration) {
  delete linear_acceleration_;
  linear_acceleration_ = linear_acceleration;
  if (linear_acceleration) {
    set_has_linear_acceleration();
  } else {
    clear_has_linear_acceleration();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.IMUSensor.linear_acceleration)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------


typedef boost::shared_ptr<gazebo::msgs::IMUSensor> IMUSensorPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::IMUSensor const> ConstIMUSensorPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_imu_5fsensor_2eproto__INCLUDED
