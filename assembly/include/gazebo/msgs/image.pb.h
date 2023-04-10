// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: image.proto

#ifndef PROTOBUF_image_2eproto__INCLUDED
#define PROTOBUF_image_2eproto__INCLUDED

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
void GZ_MSGS_VISIBLE protobuf_AddDesc_image_2eproto();
void protobuf_AssignDesc_image_2eproto();
void protobuf_ShutdownFile_image_2eproto();

class Image;

// ===================================================================

class GZ_MSGS_VISIBLE Image : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Image) */ {
 public:
  Image();
  virtual ~Image();

  Image(const Image& from);

  inline Image& operator=(const Image& from) {
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
  static const Image& default_instance();

  void Swap(Image* other);

  // implements Message ----------------------------------------------

  inline Image* New() const { return New(NULL); }

  Image* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Image& from);
  void MergeFrom(const Image& from);
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
  void InternalSwap(Image* other);
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

  // required uint32 width = 1;
  bool has_width() const;
  void clear_width();
  static const int kWidthFieldNumber = 1;
  ::google::protobuf::uint32 width() const;
  void set_width(::google::protobuf::uint32 value);

  // required uint32 height = 2;
  bool has_height() const;
  void clear_height();
  static const int kHeightFieldNumber = 2;
  ::google::protobuf::uint32 height() const;
  void set_height(::google::protobuf::uint32 value);

  // required uint32 pixel_format = 3;
  bool has_pixel_format() const;
  void clear_pixel_format();
  static const int kPixelFormatFieldNumber = 3;
  ::google::protobuf::uint32 pixel_format() const;
  void set_pixel_format(::google::protobuf::uint32 value);

  // required uint32 step = 4;
  bool has_step() const;
  void clear_step();
  static const int kStepFieldNumber = 4;
  ::google::protobuf::uint32 step() const;
  void set_step(::google::protobuf::uint32 value);

  // required bytes data = 5;
  bool has_data() const;
  void clear_data();
  static const int kDataFieldNumber = 5;
  const ::std::string& data() const;
  void set_data(const ::std::string& value);
  void set_data(const char* value);
  void set_data(const void* value, size_t size);
  ::std::string* mutable_data();
  ::std::string* release_data();
  void set_allocated_data(::std::string* data);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Image)
 private:
  inline void set_has_width();
  inline void clear_has_width();
  inline void set_has_height();
  inline void clear_has_height();
  inline void set_has_pixel_format();
  inline void clear_has_pixel_format();
  inline void set_has_step();
  inline void clear_has_step();
  inline void set_has_data();
  inline void clear_has_data();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::uint32 width_;
  ::google::protobuf::uint32 height_;
  ::google::protobuf::uint32 pixel_format_;
  ::google::protobuf::uint32 step_;
  ::google::protobuf::internal::ArenaStringPtr data_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_image_2eproto();
  friend void protobuf_AssignDesc_image_2eproto();
  friend void protobuf_ShutdownFile_image_2eproto();

  void InitAsDefaultInstance();
  static Image* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Image

// required uint32 width = 1;
inline bool Image::has_width() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Image::set_has_width() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Image::clear_has_width() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Image::clear_width() {
  width_ = 0u;
  clear_has_width();
}
inline ::google::protobuf::uint32 Image::width() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Image.width)
  return width_;
}
inline void Image::set_width(::google::protobuf::uint32 value) {
  set_has_width();
  width_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Image.width)
}

// required uint32 height = 2;
inline bool Image::has_height() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Image::set_has_height() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Image::clear_has_height() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Image::clear_height() {
  height_ = 0u;
  clear_has_height();
}
inline ::google::protobuf::uint32 Image::height() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Image.height)
  return height_;
}
inline void Image::set_height(::google::protobuf::uint32 value) {
  set_has_height();
  height_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Image.height)
}

// required uint32 pixel_format = 3;
inline bool Image::has_pixel_format() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Image::set_has_pixel_format() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Image::clear_has_pixel_format() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Image::clear_pixel_format() {
  pixel_format_ = 0u;
  clear_has_pixel_format();
}
inline ::google::protobuf::uint32 Image::pixel_format() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Image.pixel_format)
  return pixel_format_;
}
inline void Image::set_pixel_format(::google::protobuf::uint32 value) {
  set_has_pixel_format();
  pixel_format_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Image.pixel_format)
}

// required uint32 step = 4;
inline bool Image::has_step() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Image::set_has_step() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Image::clear_has_step() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Image::clear_step() {
  step_ = 0u;
  clear_has_step();
}
inline ::google::protobuf::uint32 Image::step() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Image.step)
  return step_;
}
inline void Image::set_step(::google::protobuf::uint32 value) {
  set_has_step();
  step_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Image.step)
}

// required bytes data = 5;
inline bool Image::has_data() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Image::set_has_data() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Image::clear_has_data() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Image::clear_data() {
  data_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_data();
}
inline const ::std::string& Image::data() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Image.data)
  return data_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Image::set_data(const ::std::string& value) {
  set_has_data();
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Image.data)
}
inline void Image::set_data(const char* value) {
  set_has_data();
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Image.data)
}
inline void Image::set_data(const void* value, size_t size) {
  set_has_data();
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Image.data)
}
inline ::std::string* Image::mutable_data() {
  set_has_data();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Image.data)
  return data_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Image::release_data() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Image.data)
  clear_has_data();
  return data_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Image::set_allocated_data(::std::string* data) {
  if (data != NULL) {
    set_has_data();
  } else {
    clear_has_data();
  }
  data_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), data);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Image.data)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Image> ImagePtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Image const> ConstImagePtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_image_2eproto__INCLUDED