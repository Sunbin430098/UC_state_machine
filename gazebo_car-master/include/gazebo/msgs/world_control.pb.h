// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: world_control.proto

#ifndef PROTOBUF_world_5fcontrol_2eproto__INCLUDED
#define PROTOBUF_world_5fcontrol_2eproto__INCLUDED

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
#include "world_reset.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_world_5fcontrol_2eproto();
void protobuf_AssignDesc_world_5fcontrol_2eproto();
void protobuf_ShutdownFile_world_5fcontrol_2eproto();

class WorldControl;

// ===================================================================

class GZ_MSGS_VISIBLE WorldControl : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.WorldControl) */ {
 public:
  WorldControl();
  virtual ~WorldControl();

  WorldControl(const WorldControl& from);

  inline WorldControl& operator=(const WorldControl& from) {
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
  static const WorldControl& default_instance();

  void Swap(WorldControl* other);

  // implements Message ----------------------------------------------

  inline WorldControl* New() const { return New(NULL); }

  WorldControl* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const WorldControl& from);
  void MergeFrom(const WorldControl& from);
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
  void InternalSwap(WorldControl* other);
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

  // optional bool pause = 1;
  bool has_pause() const;
  void clear_pause();
  static const int kPauseFieldNumber = 1;
  bool pause() const;
  void set_pause(bool value);

  // optional bool step = 2;
  bool has_step() const;
  void clear_step();
  static const int kStepFieldNumber = 2;
  bool step() const;
  void set_step(bool value);

  // optional uint32 multi_step = 3;
  bool has_multi_step() const;
  void clear_multi_step();
  static const int kMultiStepFieldNumber = 3;
  ::google::protobuf::uint32 multi_step() const;
  void set_multi_step(::google::protobuf::uint32 value);

  // optional .gazebo.msgs.WorldReset reset = 4;
  bool has_reset() const;
  void clear_reset();
  static const int kResetFieldNumber = 4;
  const ::gazebo::msgs::WorldReset& reset() const;
  ::gazebo::msgs::WorldReset* mutable_reset();
  ::gazebo::msgs::WorldReset* release_reset();
  void set_allocated_reset(::gazebo::msgs::WorldReset* reset);

  // optional uint32 seed = 5;
  bool has_seed() const;
  void clear_seed();
  static const int kSeedFieldNumber = 5;
  ::google::protobuf::uint32 seed() const;
  void set_seed(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.WorldControl)
 private:
  inline void set_has_pause();
  inline void clear_has_pause();
  inline void set_has_step();
  inline void clear_has_step();
  inline void set_has_multi_step();
  inline void clear_has_multi_step();
  inline void set_has_reset();
  inline void clear_has_reset();
  inline void set_has_seed();
  inline void clear_has_seed();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  bool pause_;
  bool step_;
  ::google::protobuf::uint32 multi_step_;
  ::gazebo::msgs::WorldReset* reset_;
  ::google::protobuf::uint32 seed_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_world_5fcontrol_2eproto();
  friend void protobuf_AssignDesc_world_5fcontrol_2eproto();
  friend void protobuf_ShutdownFile_world_5fcontrol_2eproto();

  void InitAsDefaultInstance();
  static WorldControl* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// WorldControl

// optional bool pause = 1;
inline bool WorldControl::has_pause() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void WorldControl::set_has_pause() {
  _has_bits_[0] |= 0x00000001u;
}
inline void WorldControl::clear_has_pause() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void WorldControl::clear_pause() {
  pause_ = false;
  clear_has_pause();
}
inline bool WorldControl::pause() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.WorldControl.pause)
  return pause_;
}
inline void WorldControl::set_pause(bool value) {
  set_has_pause();
  pause_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.WorldControl.pause)
}

// optional bool step = 2;
inline bool WorldControl::has_step() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void WorldControl::set_has_step() {
  _has_bits_[0] |= 0x00000002u;
}
inline void WorldControl::clear_has_step() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void WorldControl::clear_step() {
  step_ = false;
  clear_has_step();
}
inline bool WorldControl::step() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.WorldControl.step)
  return step_;
}
inline void WorldControl::set_step(bool value) {
  set_has_step();
  step_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.WorldControl.step)
}

// optional uint32 multi_step = 3;
inline bool WorldControl::has_multi_step() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void WorldControl::set_has_multi_step() {
  _has_bits_[0] |= 0x00000004u;
}
inline void WorldControl::clear_has_multi_step() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void WorldControl::clear_multi_step() {
  multi_step_ = 0u;
  clear_has_multi_step();
}
inline ::google::protobuf::uint32 WorldControl::multi_step() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.WorldControl.multi_step)
  return multi_step_;
}
inline void WorldControl::set_multi_step(::google::protobuf::uint32 value) {
  set_has_multi_step();
  multi_step_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.WorldControl.multi_step)
}

// optional .gazebo.msgs.WorldReset reset = 4;
inline bool WorldControl::has_reset() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void WorldControl::set_has_reset() {
  _has_bits_[0] |= 0x00000008u;
}
inline void WorldControl::clear_has_reset() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void WorldControl::clear_reset() {
  if (reset_ != NULL) reset_->::gazebo::msgs::WorldReset::Clear();
  clear_has_reset();
}
inline const ::gazebo::msgs::WorldReset& WorldControl::reset() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.WorldControl.reset)
  return reset_ != NULL ? *reset_ : *default_instance_->reset_;
}
inline ::gazebo::msgs::WorldReset* WorldControl::mutable_reset() {
  set_has_reset();
  if (reset_ == NULL) {
    reset_ = new ::gazebo::msgs::WorldReset;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.WorldControl.reset)
  return reset_;
}
inline ::gazebo::msgs::WorldReset* WorldControl::release_reset() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.WorldControl.reset)
  clear_has_reset();
  ::gazebo::msgs::WorldReset* temp = reset_;
  reset_ = NULL;
  return temp;
}
inline void WorldControl::set_allocated_reset(::gazebo::msgs::WorldReset* reset) {
  delete reset_;
  reset_ = reset;
  if (reset) {
    set_has_reset();
  } else {
    clear_has_reset();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.WorldControl.reset)
}

// optional uint32 seed = 5;
inline bool WorldControl::has_seed() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void WorldControl::set_has_seed() {
  _has_bits_[0] |= 0x00000010u;
}
inline void WorldControl::clear_has_seed() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void WorldControl::clear_seed() {
  seed_ = 0u;
  clear_has_seed();
}
inline ::google::protobuf::uint32 WorldControl::seed() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.WorldControl.seed)
  return seed_;
}
inline void WorldControl::set_seed(::google::protobuf::uint32 value) {
  set_has_seed();
  seed_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.WorldControl.seed)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::WorldControl> WorldControlPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::WorldControl const> ConstWorldControlPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_world_5fcontrol_2eproto__INCLUDED
