// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: joystick.proto

#ifndef PROTOBUF_joystick_2eproto__INCLUDED
#define PROTOBUF_joystick_2eproto__INCLUDED

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
void GZ_MSGS_VISIBLE protobuf_AddDesc_joystick_2eproto();
void protobuf_AssignDesc_joystick_2eproto();
void protobuf_ShutdownFile_joystick_2eproto();

class Joystick;

// ===================================================================

class GZ_MSGS_VISIBLE Joystick : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Joystick) */ {
 public:
  Joystick();
  virtual ~Joystick();

  Joystick(const Joystick& from);

  inline Joystick& operator=(const Joystick& from) {
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
  static const Joystick& default_instance();

  void Swap(Joystick* other);

  // implements Message ----------------------------------------------

  inline Joystick* New() const { return New(NULL); }

  Joystick* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Joystick& from);
  void MergeFrom(const Joystick& from);
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
  void InternalSwap(Joystick* other);
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

  // optional .gazebo.msgs.Vector3d translation = 1;
  bool has_translation() const;
  void clear_translation();
  static const int kTranslationFieldNumber = 1;
  const ::gazebo::msgs::Vector3d& translation() const;
  ::gazebo::msgs::Vector3d* mutable_translation();
  ::gazebo::msgs::Vector3d* release_translation();
  void set_allocated_translation(::gazebo::msgs::Vector3d* translation);

  // optional .gazebo.msgs.Vector3d rotation = 2;
  bool has_rotation() const;
  void clear_rotation();
  static const int kRotationFieldNumber = 2;
  const ::gazebo::msgs::Vector3d& rotation() const;
  ::gazebo::msgs::Vector3d* mutable_rotation();
  ::gazebo::msgs::Vector3d* release_rotation();
  void set_allocated_rotation(::gazebo::msgs::Vector3d* rotation);

  // repeated int32 buttons = 3;
  int buttons_size() const;
  void clear_buttons();
  static const int kButtonsFieldNumber = 3;
  ::google::protobuf::int32 buttons(int index) const;
  void set_buttons(int index, ::google::protobuf::int32 value);
  void add_buttons(::google::protobuf::int32 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      buttons() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_buttons();

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Joystick)
 private:
  inline void set_has_translation();
  inline void clear_has_translation();
  inline void set_has_rotation();
  inline void clear_has_rotation();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Vector3d* translation_;
  ::gazebo::msgs::Vector3d* rotation_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > buttons_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_joystick_2eproto();
  friend void protobuf_AssignDesc_joystick_2eproto();
  friend void protobuf_ShutdownFile_joystick_2eproto();

  void InitAsDefaultInstance();
  static Joystick* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Joystick

// optional .gazebo.msgs.Vector3d translation = 1;
inline bool Joystick::has_translation() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Joystick::set_has_translation() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Joystick::clear_has_translation() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Joystick::clear_translation() {
  if (translation_ != NULL) translation_->::gazebo::msgs::Vector3d::Clear();
  clear_has_translation();
}
inline const ::gazebo::msgs::Vector3d& Joystick::translation() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Joystick.translation)
  return translation_ != NULL ? *translation_ : *default_instance_->translation_;
}
inline ::gazebo::msgs::Vector3d* Joystick::mutable_translation() {
  set_has_translation();
  if (translation_ == NULL) {
    translation_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Joystick.translation)
  return translation_;
}
inline ::gazebo::msgs::Vector3d* Joystick::release_translation() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Joystick.translation)
  clear_has_translation();
  ::gazebo::msgs::Vector3d* temp = translation_;
  translation_ = NULL;
  return temp;
}
inline void Joystick::set_allocated_translation(::gazebo::msgs::Vector3d* translation) {
  delete translation_;
  translation_ = translation;
  if (translation) {
    set_has_translation();
  } else {
    clear_has_translation();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Joystick.translation)
}

// optional .gazebo.msgs.Vector3d rotation = 2;
inline bool Joystick::has_rotation() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Joystick::set_has_rotation() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Joystick::clear_has_rotation() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Joystick::clear_rotation() {
  if (rotation_ != NULL) rotation_->::gazebo::msgs::Vector3d::Clear();
  clear_has_rotation();
}
inline const ::gazebo::msgs::Vector3d& Joystick::rotation() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Joystick.rotation)
  return rotation_ != NULL ? *rotation_ : *default_instance_->rotation_;
}
inline ::gazebo::msgs::Vector3d* Joystick::mutable_rotation() {
  set_has_rotation();
  if (rotation_ == NULL) {
    rotation_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Joystick.rotation)
  return rotation_;
}
inline ::gazebo::msgs::Vector3d* Joystick::release_rotation() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Joystick.rotation)
  clear_has_rotation();
  ::gazebo::msgs::Vector3d* temp = rotation_;
  rotation_ = NULL;
  return temp;
}
inline void Joystick::set_allocated_rotation(::gazebo::msgs::Vector3d* rotation) {
  delete rotation_;
  rotation_ = rotation;
  if (rotation) {
    set_has_rotation();
  } else {
    clear_has_rotation();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Joystick.rotation)
}

// repeated int32 buttons = 3;
inline int Joystick::buttons_size() const {
  return buttons_.size();
}
inline void Joystick::clear_buttons() {
  buttons_.Clear();
}
inline ::google::protobuf::int32 Joystick::buttons(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Joystick.buttons)
  return buttons_.Get(index);
}
inline void Joystick::set_buttons(int index, ::google::protobuf::int32 value) {
  buttons_.Set(index, value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Joystick.buttons)
}
inline void Joystick::add_buttons(::google::protobuf::int32 value) {
  buttons_.Add(value);
  // @@protoc_insertion_point(field_add:gazebo.msgs.Joystick.buttons)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
Joystick::buttons() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Joystick.buttons)
  return buttons_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
Joystick::mutable_buttons() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Joystick.buttons)
  return &buttons_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Joystick> JoystickPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Joystick const> ConstJoystickPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_joystick_2eproto__INCLUDED
