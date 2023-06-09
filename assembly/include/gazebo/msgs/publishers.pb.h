// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: publishers.proto

#ifndef PROTOBUF_publishers_2eproto__INCLUDED
#define PROTOBUF_publishers_2eproto__INCLUDED

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
#include "publish.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_publishers_2eproto();
void protobuf_AssignDesc_publishers_2eproto();
void protobuf_ShutdownFile_publishers_2eproto();

class Publishers;

// ===================================================================

class GZ_MSGS_VISIBLE Publishers : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Publishers) */ {
 public:
  Publishers();
  virtual ~Publishers();

  Publishers(const Publishers& from);

  inline Publishers& operator=(const Publishers& from) {
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
  static const Publishers& default_instance();

  void Swap(Publishers* other);

  // implements Message ----------------------------------------------

  inline Publishers* New() const { return New(NULL); }

  Publishers* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Publishers& from);
  void MergeFrom(const Publishers& from);
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
  void InternalSwap(Publishers* other);
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

  // repeated .gazebo.msgs.Publish publisher = 1;
  int publisher_size() const;
  void clear_publisher();
  static const int kPublisherFieldNumber = 1;
  const ::gazebo::msgs::Publish& publisher(int index) const;
  ::gazebo::msgs::Publish* mutable_publisher(int index);
  ::gazebo::msgs::Publish* add_publisher();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Publish >*
      mutable_publisher();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Publish >&
      publisher() const;

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Publishers)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Publish > publisher_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_publishers_2eproto();
  friend void protobuf_AssignDesc_publishers_2eproto();
  friend void protobuf_ShutdownFile_publishers_2eproto();

  void InitAsDefaultInstance();
  static Publishers* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Publishers

// repeated .gazebo.msgs.Publish publisher = 1;
inline int Publishers::publisher_size() const {
  return publisher_.size();
}
inline void Publishers::clear_publisher() {
  publisher_.Clear();
}
inline const ::gazebo::msgs::Publish& Publishers::publisher(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Publishers.publisher)
  return publisher_.Get(index);
}
inline ::gazebo::msgs::Publish* Publishers::mutable_publisher(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Publishers.publisher)
  return publisher_.Mutable(index);
}
inline ::gazebo::msgs::Publish* Publishers::add_publisher() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.Publishers.publisher)
  return publisher_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Publish >*
Publishers::mutable_publisher() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Publishers.publisher)
  return &publisher_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Publish >&
Publishers::publisher() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Publishers.publisher)
  return publisher_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Publishers> PublishersPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Publishers const> ConstPublishersPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_publishers_2eproto__INCLUDED
