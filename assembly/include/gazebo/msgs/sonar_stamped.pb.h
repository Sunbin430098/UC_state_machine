// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sonar_stamped.proto

#ifndef PROTOBUF_sonar_5fstamped_2eproto__INCLUDED
#define PROTOBUF_sonar_5fstamped_2eproto__INCLUDED

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
#include "time.pb.h"
#include "sonar.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_sonar_5fstamped_2eproto();
void protobuf_AssignDesc_sonar_5fstamped_2eproto();
void protobuf_ShutdownFile_sonar_5fstamped_2eproto();

class SonarStamped;

// ===================================================================

class GZ_MSGS_VISIBLE SonarStamped : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.SonarStamped) */ {
 public:
  SonarStamped();
  virtual ~SonarStamped();

  SonarStamped(const SonarStamped& from);

  inline SonarStamped& operator=(const SonarStamped& from) {
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
  static const SonarStamped& default_instance();

  void Swap(SonarStamped* other);

  // implements Message ----------------------------------------------

  inline SonarStamped* New() const { return New(NULL); }

  SonarStamped* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const SonarStamped& from);
  void MergeFrom(const SonarStamped& from);
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
  void InternalSwap(SonarStamped* other);
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

  // required .gazebo.msgs.Time time = 1;
  bool has_time() const;
  void clear_time();
  static const int kTimeFieldNumber = 1;
  const ::gazebo::msgs::Time& time() const;
  ::gazebo::msgs::Time* mutable_time();
  ::gazebo::msgs::Time* release_time();
  void set_allocated_time(::gazebo::msgs::Time* time);

  // required .gazebo.msgs.Sonar sonar = 2;
  bool has_sonar() const;
  void clear_sonar();
  static const int kSonarFieldNumber = 2;
  const ::gazebo::msgs::Sonar& sonar() const;
  ::gazebo::msgs::Sonar* mutable_sonar();
  ::gazebo::msgs::Sonar* release_sonar();
  void set_allocated_sonar(::gazebo::msgs::Sonar* sonar);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.SonarStamped)
 private:
  inline void set_has_time();
  inline void clear_has_time();
  inline void set_has_sonar();
  inline void clear_has_sonar();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Time* time_;
  ::gazebo::msgs::Sonar* sonar_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_sonar_5fstamped_2eproto();
  friend void protobuf_AssignDesc_sonar_5fstamped_2eproto();
  friend void protobuf_ShutdownFile_sonar_5fstamped_2eproto();

  void InitAsDefaultInstance();
  static SonarStamped* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// SonarStamped

// required .gazebo.msgs.Time time = 1;
inline bool SonarStamped::has_time() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SonarStamped::set_has_time() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SonarStamped::clear_has_time() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SonarStamped::clear_time() {
  if (time_ != NULL) time_->::gazebo::msgs::Time::Clear();
  clear_has_time();
}
inline const ::gazebo::msgs::Time& SonarStamped::time() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SonarStamped.time)
  return time_ != NULL ? *time_ : *default_instance_->time_;
}
inline ::gazebo::msgs::Time* SonarStamped::mutable_time() {
  set_has_time();
  if (time_ == NULL) {
    time_ = new ::gazebo::msgs::Time;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.SonarStamped.time)
  return time_;
}
inline ::gazebo::msgs::Time* SonarStamped::release_time() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.SonarStamped.time)
  clear_has_time();
  ::gazebo::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline void SonarStamped::set_allocated_time(::gazebo::msgs::Time* time) {
  delete time_;
  time_ = time;
  if (time) {
    set_has_time();
  } else {
    clear_has_time();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.SonarStamped.time)
}

// required .gazebo.msgs.Sonar sonar = 2;
inline bool SonarStamped::has_sonar() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SonarStamped::set_has_sonar() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SonarStamped::clear_has_sonar() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SonarStamped::clear_sonar() {
  if (sonar_ != NULL) sonar_->::gazebo::msgs::Sonar::Clear();
  clear_has_sonar();
}
inline const ::gazebo::msgs::Sonar& SonarStamped::sonar() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SonarStamped.sonar)
  return sonar_ != NULL ? *sonar_ : *default_instance_->sonar_;
}
inline ::gazebo::msgs::Sonar* SonarStamped::mutable_sonar() {
  set_has_sonar();
  if (sonar_ == NULL) {
    sonar_ = new ::gazebo::msgs::Sonar;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.SonarStamped.sonar)
  return sonar_;
}
inline ::gazebo::msgs::Sonar* SonarStamped::release_sonar() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.SonarStamped.sonar)
  clear_has_sonar();
  ::gazebo::msgs::Sonar* temp = sonar_;
  sonar_ = NULL;
  return temp;
}
inline void SonarStamped::set_allocated_sonar(::gazebo::msgs::Sonar* sonar) {
  delete sonar_;
  sonar_ = sonar;
  if (sonar) {
    set_has_sonar();
  } else {
    clear_has_sonar();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.SonarStamped.sonar)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::SonarStamped> SonarStampedPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::SonarStamped const> ConstSonarStampedPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_sonar_5fstamped_2eproto__INCLUDED
