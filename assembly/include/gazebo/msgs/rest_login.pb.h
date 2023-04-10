// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: rest_login.proto

#ifndef PROTOBUF_rest_5flogin_2eproto__INCLUDED
#define PROTOBUF_rest_5flogin_2eproto__INCLUDED

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
void GZ_MSGS_VISIBLE protobuf_AddDesc_rest_5flogin_2eproto();
void protobuf_AssignDesc_rest_5flogin_2eproto();
void protobuf_ShutdownFile_rest_5flogin_2eproto();

class RestLogin;

// ===================================================================

class GZ_MSGS_VISIBLE RestLogin : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.RestLogin) */ {
 public:
  RestLogin();
  virtual ~RestLogin();

  RestLogin(const RestLogin& from);

  inline RestLogin& operator=(const RestLogin& from) {
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
  static const RestLogin& default_instance();

  void Swap(RestLogin* other);

  // implements Message ----------------------------------------------

  inline RestLogin* New() const { return New(NULL); }

  RestLogin* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RestLogin& from);
  void MergeFrom(const RestLogin& from);
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
  void InternalSwap(RestLogin* other);
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

  // optional uint32 id = 1;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 1;
  ::google::protobuf::uint32 id() const;
  void set_id(::google::protobuf::uint32 value);

  // required string url = 2;
  bool has_url() const;
  void clear_url();
  static const int kUrlFieldNumber = 2;
  const ::std::string& url() const;
  void set_url(const ::std::string& value);
  void set_url(const char* value);
  void set_url(const char* value, size_t size);
  ::std::string* mutable_url();
  ::std::string* release_url();
  void set_allocated_url(::std::string* url);

  // optional string username = 3;
  bool has_username() const;
  void clear_username();
  static const int kUsernameFieldNumber = 3;
  const ::std::string& username() const;
  void set_username(const ::std::string& value);
  void set_username(const char* value);
  void set_username(const char* value, size_t size);
  ::std::string* mutable_username();
  ::std::string* release_username();
  void set_allocated_username(::std::string* username);

  // optional string password = 4;
  bool has_password() const;
  void clear_password();
  static const int kPasswordFieldNumber = 4;
  const ::std::string& password() const;
  void set_password(const ::std::string& value);
  void set_password(const char* value);
  void set_password(const char* value, size_t size);
  ::std::string* mutable_password();
  ::std::string* release_password();
  void set_allocated_password(::std::string* password);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.RestLogin)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_url();
  inline void clear_has_url();
  inline void set_has_username();
  inline void clear_has_username();
  inline void set_has_password();
  inline void clear_has_password();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr url_;
  ::google::protobuf::internal::ArenaStringPtr username_;
  ::google::protobuf::internal::ArenaStringPtr password_;
  ::google::protobuf::uint32 id_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_rest_5flogin_2eproto();
  friend void protobuf_AssignDesc_rest_5flogin_2eproto();
  friend void protobuf_ShutdownFile_rest_5flogin_2eproto();

  void InitAsDefaultInstance();
  static RestLogin* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// RestLogin

// optional uint32 id = 1;
inline bool RestLogin::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RestLogin::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RestLogin::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RestLogin::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 RestLogin::id() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.RestLogin.id)
  return id_;
}
inline void RestLogin::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.RestLogin.id)
}

// required string url = 2;
inline bool RestLogin::has_url() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RestLogin::set_has_url() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RestLogin::clear_has_url() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void RestLogin::clear_url() {
  url_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_url();
}
inline const ::std::string& RestLogin::url() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.RestLogin.url)
  return url_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RestLogin::set_url(const ::std::string& value) {
  set_has_url();
  url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.RestLogin.url)
}
inline void RestLogin::set_url(const char* value) {
  set_has_url();
  url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.RestLogin.url)
}
inline void RestLogin::set_url(const char* value, size_t size) {
  set_has_url();
  url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.RestLogin.url)
}
inline ::std::string* RestLogin::mutable_url() {
  set_has_url();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.RestLogin.url)
  return url_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* RestLogin::release_url() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.RestLogin.url)
  clear_has_url();
  return url_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RestLogin::set_allocated_url(::std::string* url) {
  if (url != NULL) {
    set_has_url();
  } else {
    clear_has_url();
  }
  url_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), url);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.RestLogin.url)
}

// optional string username = 3;
inline bool RestLogin::has_username() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void RestLogin::set_has_username() {
  _has_bits_[0] |= 0x00000004u;
}
inline void RestLogin::clear_has_username() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void RestLogin::clear_username() {
  username_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_username();
}
inline const ::std::string& RestLogin::username() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.RestLogin.username)
  return username_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RestLogin::set_username(const ::std::string& value) {
  set_has_username();
  username_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.RestLogin.username)
}
inline void RestLogin::set_username(const char* value) {
  set_has_username();
  username_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.RestLogin.username)
}
inline void RestLogin::set_username(const char* value, size_t size) {
  set_has_username();
  username_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.RestLogin.username)
}
inline ::std::string* RestLogin::mutable_username() {
  set_has_username();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.RestLogin.username)
  return username_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* RestLogin::release_username() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.RestLogin.username)
  clear_has_username();
  return username_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RestLogin::set_allocated_username(::std::string* username) {
  if (username != NULL) {
    set_has_username();
  } else {
    clear_has_username();
  }
  username_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), username);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.RestLogin.username)
}

// optional string password = 4;
inline bool RestLogin::has_password() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void RestLogin::set_has_password() {
  _has_bits_[0] |= 0x00000008u;
}
inline void RestLogin::clear_has_password() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void RestLogin::clear_password() {
  password_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_password();
}
inline const ::std::string& RestLogin::password() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.RestLogin.password)
  return password_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RestLogin::set_password(const ::std::string& value) {
  set_has_password();
  password_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.RestLogin.password)
}
inline void RestLogin::set_password(const char* value) {
  set_has_password();
  password_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.RestLogin.password)
}
inline void RestLogin::set_password(const char* value, size_t size) {
  set_has_password();
  password_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.RestLogin.password)
}
inline ::std::string* RestLogin::mutable_password() {
  set_has_password();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.RestLogin.password)
  return password_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* RestLogin::release_password() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.RestLogin.password)
  clear_has_password();
  return password_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RestLogin::set_allocated_password(::std::string* password) {
  if (password != NULL) {
    set_has_password();
  } else {
    clear_has_password();
  }
  password_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), password);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.RestLogin.password)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::RestLogin> RestLoginPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::RestLogin const> ConstRestLoginPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_rest_5flogin_2eproto__INCLUDED