// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: response.proto

#ifndef PROTOBUF_response_2eproto__INCLUDED
#define PROTOBUF_response_2eproto__INCLUDED

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
void GZ_MSGS_VISIBLE protobuf_AddDesc_response_2eproto();
void protobuf_AssignDesc_response_2eproto();
void protobuf_ShutdownFile_response_2eproto();

class Response;

// ===================================================================

class GZ_MSGS_VISIBLE Response : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Response) */ {
 public:
  Response();
  virtual ~Response();

  Response(const Response& from);

  inline Response& operator=(const Response& from) {
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
  static const Response& default_instance();

  void Swap(Response* other);

  // implements Message ----------------------------------------------

  inline Response* New() const { return New(NULL); }

  Response* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Response& from);
  void MergeFrom(const Response& from);
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
  void InternalSwap(Response* other);
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

  // required int32 id = 1;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 1;
  ::google::protobuf::int32 id() const;
  void set_id(::google::protobuf::int32 value);

  // required string request = 2;
  bool has_request() const;
  void clear_request();
  static const int kRequestFieldNumber = 2;
  const ::std::string& request() const;
  void set_request(const ::std::string& value);
  void set_request(const char* value);
  void set_request(const char* value, size_t size);
  ::std::string* mutable_request();
  ::std::string* release_request();
  void set_allocated_request(::std::string* request);

  // required string response = 3;
  bool has_response() const;
  void clear_response();
  static const int kResponseFieldNumber = 3;
  const ::std::string& response() const;
  void set_response(const ::std::string& value);
  void set_response(const char* value);
  void set_response(const char* value, size_t size);
  ::std::string* mutable_response();
  ::std::string* release_response();
  void set_allocated_response(::std::string* response);

  // optional string type = 4;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 4;
  const ::std::string& type() const;
  void set_type(const ::std::string& value);
  void set_type(const char* value);
  void set_type(const char* value, size_t size);
  ::std::string* mutable_type();
  ::std::string* release_type();
  void set_allocated_type(::std::string* type);

  // optional bytes serialized_data = 5;
  bool has_serialized_data() const;
  void clear_serialized_data();
  static const int kSerializedDataFieldNumber = 5;
  const ::std::string& serialized_data() const;
  void set_serialized_data(const ::std::string& value);
  void set_serialized_data(const char* value);
  void set_serialized_data(const void* value, size_t size);
  ::std::string* mutable_serialized_data();
  ::std::string* release_serialized_data();
  void set_allocated_serialized_data(::std::string* serialized_data);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Response)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_request();
  inline void clear_has_request();
  inline void set_has_response();
  inline void clear_has_response();
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_serialized_data();
  inline void clear_has_serialized_data();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr request_;
  ::google::protobuf::internal::ArenaStringPtr response_;
  ::google::protobuf::internal::ArenaStringPtr type_;
  ::google::protobuf::internal::ArenaStringPtr serialized_data_;
  ::google::protobuf::int32 id_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_response_2eproto();
  friend void protobuf_AssignDesc_response_2eproto();
  friend void protobuf_ShutdownFile_response_2eproto();

  void InitAsDefaultInstance();
  static Response* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Response

// required int32 id = 1;
inline bool Response::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Response::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Response::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Response::clear_id() {
  id_ = 0;
  clear_has_id();
}
inline ::google::protobuf::int32 Response::id() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Response.id)
  return id_;
}
inline void Response::set_id(::google::protobuf::int32 value) {
  set_has_id();
  id_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Response.id)
}

// required string request = 2;
inline bool Response::has_request() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Response::set_has_request() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Response::clear_has_request() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Response::clear_request() {
  request_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_request();
}
inline const ::std::string& Response::request() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Response.request)
  return request_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_request(const ::std::string& value) {
  set_has_request();
  request_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Response.request)
}
inline void Response::set_request(const char* value) {
  set_has_request();
  request_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Response.request)
}
inline void Response::set_request(const char* value, size_t size) {
  set_has_request();
  request_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Response.request)
}
inline ::std::string* Response::mutable_request() {
  set_has_request();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Response.request)
  return request_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Response::release_request() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Response.request)
  clear_has_request();
  return request_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_allocated_request(::std::string* request) {
  if (request != NULL) {
    set_has_request();
  } else {
    clear_has_request();
  }
  request_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), request);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Response.request)
}

// required string response = 3;
inline bool Response::has_response() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Response::set_has_response() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Response::clear_has_response() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Response::clear_response() {
  response_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_response();
}
inline const ::std::string& Response::response() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Response.response)
  return response_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_response(const ::std::string& value) {
  set_has_response();
  response_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Response.response)
}
inline void Response::set_response(const char* value) {
  set_has_response();
  response_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Response.response)
}
inline void Response::set_response(const char* value, size_t size) {
  set_has_response();
  response_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Response.response)
}
inline ::std::string* Response::mutable_response() {
  set_has_response();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Response.response)
  return response_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Response::release_response() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Response.response)
  clear_has_response();
  return response_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_allocated_response(::std::string* response) {
  if (response != NULL) {
    set_has_response();
  } else {
    clear_has_response();
  }
  response_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), response);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Response.response)
}

// optional string type = 4;
inline bool Response::has_type() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Response::set_has_type() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Response::clear_has_type() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Response::clear_type() {
  type_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_type();
}
inline const ::std::string& Response::type() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Response.type)
  return type_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_type(const ::std::string& value) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Response.type)
}
inline void Response::set_type(const char* value) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Response.type)
}
inline void Response::set_type(const char* value, size_t size) {
  set_has_type();
  type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Response.type)
}
inline ::std::string* Response::mutable_type() {
  set_has_type();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Response.type)
  return type_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Response::release_type() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Response.type)
  clear_has_type();
  return type_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_allocated_type(::std::string* type) {
  if (type != NULL) {
    set_has_type();
  } else {
    clear_has_type();
  }
  type_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), type);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Response.type)
}

// optional bytes serialized_data = 5;
inline bool Response::has_serialized_data() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Response::set_has_serialized_data() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Response::clear_has_serialized_data() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Response::clear_serialized_data() {
  serialized_data_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_serialized_data();
}
inline const ::std::string& Response::serialized_data() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Response.serialized_data)
  return serialized_data_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_serialized_data(const ::std::string& value) {
  set_has_serialized_data();
  serialized_data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Response.serialized_data)
}
inline void Response::set_serialized_data(const char* value) {
  set_has_serialized_data();
  serialized_data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Response.serialized_data)
}
inline void Response::set_serialized_data(const void* value, size_t size) {
  set_has_serialized_data();
  serialized_data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Response.serialized_data)
}
inline ::std::string* Response::mutable_serialized_data() {
  set_has_serialized_data();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Response.serialized_data)
  return serialized_data_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Response::release_serialized_data() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Response.serialized_data)
  clear_has_serialized_data();
  return serialized_data_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Response::set_allocated_serialized_data(::std::string* serialized_data) {
  if (serialized_data != NULL) {
    set_has_serialized_data();
  } else {
    clear_has_serialized_data();
  }
  serialized_data_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), serialized_data);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Response.serialized_data)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Response> ResponsePtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Response const> ConstResponsePtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_response_2eproto__INCLUDED
