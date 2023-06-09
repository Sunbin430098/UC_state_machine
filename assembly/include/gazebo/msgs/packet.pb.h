// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: packet.proto

#ifndef PROTOBUF_packet_2eproto__INCLUDED
#define PROTOBUF_packet_2eproto__INCLUDED

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
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_packet_2eproto();
void protobuf_AssignDesc_packet_2eproto();
void protobuf_ShutdownFile_packet_2eproto();

class Packet;

// ===================================================================

class GZ_MSGS_VISIBLE Packet : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Packet) */ {
 public:
  Packet();
  virtual ~Packet();

  Packet(const Packet& from);

  inline Packet& operator=(const Packet& from) {
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
  static const Packet& default_instance();

  void Swap(Packet* other);

  // implements Message ----------------------------------------------

  inline Packet* New() const { return New(NULL); }

  Packet* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Packet& from);
  void MergeFrom(const Packet& from);
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
  void InternalSwap(Packet* other);
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

  // required .gazebo.msgs.Time stamp = 1;
  bool has_stamp() const;
  void clear_stamp();
  static const int kStampFieldNumber = 1;
  const ::gazebo::msgs::Time& stamp() const;
  ::gazebo::msgs::Time* mutable_stamp();
  ::gazebo::msgs::Time* release_stamp();
  void set_allocated_stamp(::gazebo::msgs::Time* stamp);

  // required string type = 2;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  const ::std::string& type() const;
  void set_type(const ::std::string& value);
  void set_type(const char* value);
  void set_type(const char* value, size_t size);
  ::std::string* mutable_type();
  ::std::string* release_type();
  void set_allocated_type(::std::string* type);

  // required bytes serialized_data = 3;
  bool has_serialized_data() const;
  void clear_serialized_data();
  static const int kSerializedDataFieldNumber = 3;
  const ::std::string& serialized_data() const;
  void set_serialized_data(const ::std::string& value);
  void set_serialized_data(const char* value);
  void set_serialized_data(const void* value, size_t size);
  ::std::string* mutable_serialized_data();
  ::std::string* release_serialized_data();
  void set_allocated_serialized_data(::std::string* serialized_data);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Packet)
 private:
  inline void set_has_stamp();
  inline void clear_has_stamp();
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_serialized_data();
  inline void clear_has_serialized_data();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Time* stamp_;
  ::google::protobuf::internal::ArenaStringPtr type_;
  ::google::protobuf::internal::ArenaStringPtr serialized_data_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_packet_2eproto();
  friend void protobuf_AssignDesc_packet_2eproto();
  friend void protobuf_ShutdownFile_packet_2eproto();

  void InitAsDefaultInstance();
  static Packet* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Packet

// required .gazebo.msgs.Time stamp = 1;
inline bool Packet::has_stamp() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Packet::set_has_stamp() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Packet::clear_has_stamp() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Packet::clear_stamp() {
  if (stamp_ != NULL) stamp_->::gazebo::msgs::Time::Clear();
  clear_has_stamp();
}
inline const ::gazebo::msgs::Time& Packet::stamp() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Packet.stamp)
  return stamp_ != NULL ? *stamp_ : *default_instance_->stamp_;
}
inline ::gazebo::msgs::Time* Packet::mutable_stamp() {
  set_has_stamp();
  if (stamp_ == NULL) {
    stamp_ = new ::gazebo::msgs::Time;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Packet.stamp)
  return stamp_;
}
inline ::gazebo::msgs::Time* Packet::release_stamp() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Packet.stamp)
  clear_has_stamp();
  ::gazebo::msgs::Time* temp = stamp_;
  stamp_ = NULL;
  return temp;
}
inline void Packet::set_allocated_stamp(::gazebo::msgs::Time* stamp) {
  delete stamp_;
  stamp_ = stamp;
  if (stamp) {
    set_has_stamp();
  } else {
    clear_has_stamp();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Packet.stamp)
}

// required string type = 2;
inline bool Packet::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Packet::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Packet::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Packet::clear_type() {
  type_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_type();
}
inline const ::std::string& Packet::type() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Packet.type)
  return type_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Packet::set_type(const ::std::string& value) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Packet.type)
}
inline void Packet::set_type(const char* value) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Packet.type)
}
inline void Packet::set_type(const char* value, size_t size) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Packet.type)
}
inline ::std::string* Packet::mutable_type() {
  set_has_type();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Packet.type)
  return type_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Packet::release_type() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Packet.type)
  clear_has_type();
  return type_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Packet::set_allocated_type(::std::string* type) {
  if (type != NULL) {
    set_has_type();
  } else {
    clear_has_type();
  }
  type_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), type);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Packet.type)
}

// required bytes serialized_data = 3;
inline bool Packet::has_serialized_data() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Packet::set_has_serialized_data() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Packet::clear_has_serialized_data() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Packet::clear_serialized_data() {
  serialized_data_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_serialized_data();
}
inline const ::std::string& Packet::serialized_data() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Packet.serialized_data)
  return serialized_data_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Packet::set_serialized_data(const ::std::string& value) {
  set_has_serialized_data();
  serialized_data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Packet.serialized_data)
}
inline void Packet::set_serialized_data(const char* value) {
  set_has_serialized_data();
  serialized_data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Packet.serialized_data)
}
inline void Packet::set_serialized_data(const void* value, size_t size) {
  set_has_serialized_data();
  serialized_data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Packet.serialized_data)
}
inline ::std::string* Packet::mutable_serialized_data() {
  set_has_serialized_data();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Packet.serialized_data)
  return serialized_data_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Packet::release_serialized_data() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Packet.serialized_data)
  clear_has_serialized_data();
  return serialized_data_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Packet::set_allocated_serialized_data(::std::string* serialized_data) {
  if (serialized_data != NULL) {
    set_has_serialized_data();
  } else {
    clear_has_serialized_data();
  }
  serialized_data_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), serialized_data);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Packet.serialized_data)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Packet> PacketPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Packet const> ConstPacketPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_packet_2eproto__INCLUDED
