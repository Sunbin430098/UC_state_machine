// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cessna.proto

#ifndef PROTOBUF_cessna_2eproto__INCLUDED
#define PROTOBUF_cessna_2eproto__INCLUDED

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
void GZ_MSGS_VISIBLE protobuf_AddDesc_cessna_2eproto();
void protobuf_AssignDesc_cessna_2eproto();
void protobuf_ShutdownFile_cessna_2eproto();

class Cessna;

// ===================================================================

class GZ_MSGS_VISIBLE Cessna : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Cessna) */ {
 public:
  Cessna();
  virtual ~Cessna();

  Cessna(const Cessna& from);

  inline Cessna& operator=(const Cessna& from) {
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
  static const Cessna& default_instance();

  void Swap(Cessna* other);

  // implements Message ----------------------------------------------

  inline Cessna* New() const { return New(NULL); }

  Cessna* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Cessna& from);
  void MergeFrom(const Cessna& from);
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
  void InternalSwap(Cessna* other);
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

  // optional float propeller_speed = 1;
  bool has_propeller_speed() const;
  void clear_propeller_speed();
  static const int kPropellerSpeedFieldNumber = 1;
  float propeller_speed() const;
  void set_propeller_speed(float value);

  // optional float left_aileron = 2;
  bool has_left_aileron() const;
  void clear_left_aileron();
  static const int kLeftAileronFieldNumber = 2;
  float left_aileron() const;
  void set_left_aileron(float value);

  // optional float left_flap = 3;
  bool has_left_flap() const;
  void clear_left_flap();
  static const int kLeftFlapFieldNumber = 3;
  float left_flap() const;
  void set_left_flap(float value);

  // optional float right_aileron = 4;
  bool has_right_aileron() const;
  void clear_right_aileron();
  static const int kRightAileronFieldNumber = 4;
  float right_aileron() const;
  void set_right_aileron(float value);

  // optional float right_flap = 5;
  bool has_right_flap() const;
  void clear_right_flap();
  static const int kRightFlapFieldNumber = 5;
  float right_flap() const;
  void set_right_flap(float value);

  // optional float elevators = 6;
  bool has_elevators() const;
  void clear_elevators();
  static const int kElevatorsFieldNumber = 6;
  float elevators() const;
  void set_elevators(float value);

  // optional float rudder = 7;
  bool has_rudder() const;
  void clear_rudder();
  static const int kRudderFieldNumber = 7;
  float rudder() const;
  void set_rudder(float value);

  // optional float cmd_propeller_speed = 8;
  bool has_cmd_propeller_speed() const;
  void clear_cmd_propeller_speed();
  static const int kCmdPropellerSpeedFieldNumber = 8;
  float cmd_propeller_speed() const;
  void set_cmd_propeller_speed(float value);

  // optional float cmd_left_aileron = 9;
  bool has_cmd_left_aileron() const;
  void clear_cmd_left_aileron();
  static const int kCmdLeftAileronFieldNumber = 9;
  float cmd_left_aileron() const;
  void set_cmd_left_aileron(float value);

  // optional float cmd_left_flap = 10;
  bool has_cmd_left_flap() const;
  void clear_cmd_left_flap();
  static const int kCmdLeftFlapFieldNumber = 10;
  float cmd_left_flap() const;
  void set_cmd_left_flap(float value);

  // optional float cmd_right_aileron = 11;
  bool has_cmd_right_aileron() const;
  void clear_cmd_right_aileron();
  static const int kCmdRightAileronFieldNumber = 11;
  float cmd_right_aileron() const;
  void set_cmd_right_aileron(float value);

  // optional float cmd_right_flap = 12;
  bool has_cmd_right_flap() const;
  void clear_cmd_right_flap();
  static const int kCmdRightFlapFieldNumber = 12;
  float cmd_right_flap() const;
  void set_cmd_right_flap(float value);

  // optional float cmd_elevators = 13;
  bool has_cmd_elevators() const;
  void clear_cmd_elevators();
  static const int kCmdElevatorsFieldNumber = 13;
  float cmd_elevators() const;
  void set_cmd_elevators(float value);

  // optional float cmd_rudder = 14;
  bool has_cmd_rudder() const;
  void clear_cmd_rudder();
  static const int kCmdRudderFieldNumber = 14;
  float cmd_rudder() const;
  void set_cmd_rudder(float value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Cessna)
 private:
  inline void set_has_propeller_speed();
  inline void clear_has_propeller_speed();
  inline void set_has_left_aileron();
  inline void clear_has_left_aileron();
  inline void set_has_left_flap();
  inline void clear_has_left_flap();
  inline void set_has_right_aileron();
  inline void clear_has_right_aileron();
  inline void set_has_right_flap();
  inline void clear_has_right_flap();
  inline void set_has_elevators();
  inline void clear_has_elevators();
  inline void set_has_rudder();
  inline void clear_has_rudder();
  inline void set_has_cmd_propeller_speed();
  inline void clear_has_cmd_propeller_speed();
  inline void set_has_cmd_left_aileron();
  inline void clear_has_cmd_left_aileron();
  inline void set_has_cmd_left_flap();
  inline void clear_has_cmd_left_flap();
  inline void set_has_cmd_right_aileron();
  inline void clear_has_cmd_right_aileron();
  inline void set_has_cmd_right_flap();
  inline void clear_has_cmd_right_flap();
  inline void set_has_cmd_elevators();
  inline void clear_has_cmd_elevators();
  inline void set_has_cmd_rudder();
  inline void clear_has_cmd_rudder();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  float propeller_speed_;
  float left_aileron_;
  float left_flap_;
  float right_aileron_;
  float right_flap_;
  float elevators_;
  float rudder_;
  float cmd_propeller_speed_;
  float cmd_left_aileron_;
  float cmd_left_flap_;
  float cmd_right_aileron_;
  float cmd_right_flap_;
  float cmd_elevators_;
  float cmd_rudder_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_cessna_2eproto();
  friend void protobuf_AssignDesc_cessna_2eproto();
  friend void protobuf_ShutdownFile_cessna_2eproto();

  void InitAsDefaultInstance();
  static Cessna* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Cessna

// optional float propeller_speed = 1;
inline bool Cessna::has_propeller_speed() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Cessna::set_has_propeller_speed() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Cessna::clear_has_propeller_speed() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Cessna::clear_propeller_speed() {
  propeller_speed_ = 0;
  clear_has_propeller_speed();
}
inline float Cessna::propeller_speed() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.propeller_speed)
  return propeller_speed_;
}
inline void Cessna::set_propeller_speed(float value) {
  set_has_propeller_speed();
  propeller_speed_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.propeller_speed)
}

// optional float left_aileron = 2;
inline bool Cessna::has_left_aileron() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Cessna::set_has_left_aileron() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Cessna::clear_has_left_aileron() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Cessna::clear_left_aileron() {
  left_aileron_ = 0;
  clear_has_left_aileron();
}
inline float Cessna::left_aileron() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.left_aileron)
  return left_aileron_;
}
inline void Cessna::set_left_aileron(float value) {
  set_has_left_aileron();
  left_aileron_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.left_aileron)
}

// optional float left_flap = 3;
inline bool Cessna::has_left_flap() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Cessna::set_has_left_flap() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Cessna::clear_has_left_flap() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Cessna::clear_left_flap() {
  left_flap_ = 0;
  clear_has_left_flap();
}
inline float Cessna::left_flap() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.left_flap)
  return left_flap_;
}
inline void Cessna::set_left_flap(float value) {
  set_has_left_flap();
  left_flap_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.left_flap)
}

// optional float right_aileron = 4;
inline bool Cessna::has_right_aileron() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Cessna::set_has_right_aileron() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Cessna::clear_has_right_aileron() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Cessna::clear_right_aileron() {
  right_aileron_ = 0;
  clear_has_right_aileron();
}
inline float Cessna::right_aileron() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.right_aileron)
  return right_aileron_;
}
inline void Cessna::set_right_aileron(float value) {
  set_has_right_aileron();
  right_aileron_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.right_aileron)
}

// optional float right_flap = 5;
inline bool Cessna::has_right_flap() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Cessna::set_has_right_flap() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Cessna::clear_has_right_flap() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Cessna::clear_right_flap() {
  right_flap_ = 0;
  clear_has_right_flap();
}
inline float Cessna::right_flap() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.right_flap)
  return right_flap_;
}
inline void Cessna::set_right_flap(float value) {
  set_has_right_flap();
  right_flap_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.right_flap)
}

// optional float elevators = 6;
inline bool Cessna::has_elevators() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Cessna::set_has_elevators() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Cessna::clear_has_elevators() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Cessna::clear_elevators() {
  elevators_ = 0;
  clear_has_elevators();
}
inline float Cessna::elevators() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.elevators)
  return elevators_;
}
inline void Cessna::set_elevators(float value) {
  set_has_elevators();
  elevators_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.elevators)
}

// optional float rudder = 7;
inline bool Cessna::has_rudder() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Cessna::set_has_rudder() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Cessna::clear_has_rudder() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Cessna::clear_rudder() {
  rudder_ = 0;
  clear_has_rudder();
}
inline float Cessna::rudder() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.rudder)
  return rudder_;
}
inline void Cessna::set_rudder(float value) {
  set_has_rudder();
  rudder_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.rudder)
}

// optional float cmd_propeller_speed = 8;
inline bool Cessna::has_cmd_propeller_speed() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Cessna::set_has_cmd_propeller_speed() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Cessna::clear_has_cmd_propeller_speed() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Cessna::clear_cmd_propeller_speed() {
  cmd_propeller_speed_ = 0;
  clear_has_cmd_propeller_speed();
}
inline float Cessna::cmd_propeller_speed() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_propeller_speed)
  return cmd_propeller_speed_;
}
inline void Cessna::set_cmd_propeller_speed(float value) {
  set_has_cmd_propeller_speed();
  cmd_propeller_speed_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_propeller_speed)
}

// optional float cmd_left_aileron = 9;
inline bool Cessna::has_cmd_left_aileron() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void Cessna::set_has_cmd_left_aileron() {
  _has_bits_[0] |= 0x00000100u;
}
inline void Cessna::clear_has_cmd_left_aileron() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void Cessna::clear_cmd_left_aileron() {
  cmd_left_aileron_ = 0;
  clear_has_cmd_left_aileron();
}
inline float Cessna::cmd_left_aileron() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_left_aileron)
  return cmd_left_aileron_;
}
inline void Cessna::set_cmd_left_aileron(float value) {
  set_has_cmd_left_aileron();
  cmd_left_aileron_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_left_aileron)
}

// optional float cmd_left_flap = 10;
inline bool Cessna::has_cmd_left_flap() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void Cessna::set_has_cmd_left_flap() {
  _has_bits_[0] |= 0x00000200u;
}
inline void Cessna::clear_has_cmd_left_flap() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void Cessna::clear_cmd_left_flap() {
  cmd_left_flap_ = 0;
  clear_has_cmd_left_flap();
}
inline float Cessna::cmd_left_flap() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_left_flap)
  return cmd_left_flap_;
}
inline void Cessna::set_cmd_left_flap(float value) {
  set_has_cmd_left_flap();
  cmd_left_flap_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_left_flap)
}

// optional float cmd_right_aileron = 11;
inline bool Cessna::has_cmd_right_aileron() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void Cessna::set_has_cmd_right_aileron() {
  _has_bits_[0] |= 0x00000400u;
}
inline void Cessna::clear_has_cmd_right_aileron() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void Cessna::clear_cmd_right_aileron() {
  cmd_right_aileron_ = 0;
  clear_has_cmd_right_aileron();
}
inline float Cessna::cmd_right_aileron() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_right_aileron)
  return cmd_right_aileron_;
}
inline void Cessna::set_cmd_right_aileron(float value) {
  set_has_cmd_right_aileron();
  cmd_right_aileron_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_right_aileron)
}

// optional float cmd_right_flap = 12;
inline bool Cessna::has_cmd_right_flap() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void Cessna::set_has_cmd_right_flap() {
  _has_bits_[0] |= 0x00000800u;
}
inline void Cessna::clear_has_cmd_right_flap() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void Cessna::clear_cmd_right_flap() {
  cmd_right_flap_ = 0;
  clear_has_cmd_right_flap();
}
inline float Cessna::cmd_right_flap() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_right_flap)
  return cmd_right_flap_;
}
inline void Cessna::set_cmd_right_flap(float value) {
  set_has_cmd_right_flap();
  cmd_right_flap_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_right_flap)
}

// optional float cmd_elevators = 13;
inline bool Cessna::has_cmd_elevators() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void Cessna::set_has_cmd_elevators() {
  _has_bits_[0] |= 0x00001000u;
}
inline void Cessna::clear_has_cmd_elevators() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void Cessna::clear_cmd_elevators() {
  cmd_elevators_ = 0;
  clear_has_cmd_elevators();
}
inline float Cessna::cmd_elevators() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_elevators)
  return cmd_elevators_;
}
inline void Cessna::set_cmd_elevators(float value) {
  set_has_cmd_elevators();
  cmd_elevators_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_elevators)
}

// optional float cmd_rudder = 14;
inline bool Cessna::has_cmd_rudder() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void Cessna::set_has_cmd_rudder() {
  _has_bits_[0] |= 0x00002000u;
}
inline void Cessna::clear_has_cmd_rudder() {
  _has_bits_[0] &= ~0x00002000u;
}
inline void Cessna::clear_cmd_rudder() {
  cmd_rudder_ = 0;
  clear_has_cmd_rudder();
}
inline float Cessna::cmd_rudder() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Cessna.cmd_rudder)
  return cmd_rudder_;
}
inline void Cessna::set_cmd_rudder(float value) {
  set_has_cmd_rudder();
  cmd_rudder_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Cessna.cmd_rudder)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Cessna> CessnaPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Cessna const> ConstCessnaPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cessna_2eproto__INCLUDED
