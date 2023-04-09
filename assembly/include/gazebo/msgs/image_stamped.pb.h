// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: image_stamped.proto

#ifndef PROTOBUF_image_5fstamped_2eproto__INCLUDED
#define PROTOBUF_image_5fstamped_2eproto__INCLUDED

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
#include "image.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_image_5fstamped_2eproto();
void protobuf_AssignDesc_image_5fstamped_2eproto();
void protobuf_ShutdownFile_image_5fstamped_2eproto();

class ImageStamped;

// ===================================================================

class GZ_MSGS_VISIBLE ImageStamped : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.ImageStamped) */ {
 public:
  ImageStamped();
  virtual ~ImageStamped();

  ImageStamped(const ImageStamped& from);

  inline ImageStamped& operator=(const ImageStamped& from) {
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
  static const ImageStamped& default_instance();

  void Swap(ImageStamped* other);

  // implements Message ----------------------------------------------

  inline ImageStamped* New() const { return New(NULL); }

  ImageStamped* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ImageStamped& from);
  void MergeFrom(const ImageStamped& from);
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
  void InternalSwap(ImageStamped* other);
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

  // required .gazebo.msgs.Image image = 2;
  bool has_image() const;
  void clear_image();
  static const int kImageFieldNumber = 2;
  const ::gazebo::msgs::Image& image() const;
  ::gazebo::msgs::Image* mutable_image();
  ::gazebo::msgs::Image* release_image();
  void set_allocated_image(::gazebo::msgs::Image* image);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.ImageStamped)
 private:
  inline void set_has_time();
  inline void clear_has_time();
  inline void set_has_image();
  inline void clear_has_image();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Time* time_;
  ::gazebo::msgs::Image* image_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_image_5fstamped_2eproto();
  friend void protobuf_AssignDesc_image_5fstamped_2eproto();
  friend void protobuf_ShutdownFile_image_5fstamped_2eproto();

  void InitAsDefaultInstance();
  static ImageStamped* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// ImageStamped

// required .gazebo.msgs.Time time = 1;
inline bool ImageStamped::has_time() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ImageStamped::set_has_time() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ImageStamped::clear_has_time() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ImageStamped::clear_time() {
  if (time_ != NULL) time_->::gazebo::msgs::Time::Clear();
  clear_has_time();
}
inline const ::gazebo::msgs::Time& ImageStamped::time() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageStamped.time)
  return time_ != NULL ? *time_ : *default_instance_->time_;
}
inline ::gazebo::msgs::Time* ImageStamped::mutable_time() {
  set_has_time();
  if (time_ == NULL) {
    time_ = new ::gazebo::msgs::Time;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.ImageStamped.time)
  return time_;
}
inline ::gazebo::msgs::Time* ImageStamped::release_time() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.ImageStamped.time)
  clear_has_time();
  ::gazebo::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline void ImageStamped::set_allocated_time(::gazebo::msgs::Time* time) {
  delete time_;
  time_ = time;
  if (time) {
    set_has_time();
  } else {
    clear_has_time();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.ImageStamped.time)
}

// required .gazebo.msgs.Image image = 2;
inline bool ImageStamped::has_image() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ImageStamped::set_has_image() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ImageStamped::clear_has_image() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ImageStamped::clear_image() {
  if (image_ != NULL) image_->::gazebo::msgs::Image::Clear();
  clear_has_image();
}
inline const ::gazebo::msgs::Image& ImageStamped::image() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageStamped.image)
  return image_ != NULL ? *image_ : *default_instance_->image_;
}
inline ::gazebo::msgs::Image* ImageStamped::mutable_image() {
  set_has_image();
  if (image_ == NULL) {
    image_ = new ::gazebo::msgs::Image;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.ImageStamped.image)
  return image_;
}
inline ::gazebo::msgs::Image* ImageStamped::release_image() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.ImageStamped.image)
  clear_has_image();
  ::gazebo::msgs::Image* temp = image_;
  image_ = NULL;
  return temp;
}
inline void ImageStamped::set_allocated_image(::gazebo::msgs::Image* image) {
  delete image_;
  image_ = image;
  if (image) {
    set_has_image();
  } else {
    clear_has_image();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.ImageStamped.image)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::ImageStamped> ImageStampedPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::ImageStamped const> ConstImageStampedPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_image_5fstamped_2eproto__INCLUDED
