// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: heightmapgeom.proto

#ifndef PROTOBUF_heightmapgeom_2eproto__INCLUDED
#define PROTOBUF_heightmapgeom_2eproto__INCLUDED

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
#include "image.pb.h"
#include "vector3d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_heightmapgeom_2eproto();
void protobuf_AssignDesc_heightmapgeom_2eproto();
void protobuf_ShutdownFile_heightmapgeom_2eproto();

class HeightmapGeom;
class HeightmapGeom_Blend;
class HeightmapGeom_Texture;

// ===================================================================

class GZ_MSGS_VISIBLE HeightmapGeom_Texture : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.HeightmapGeom.Texture) */ {
 public:
  HeightmapGeom_Texture();
  virtual ~HeightmapGeom_Texture();

  HeightmapGeom_Texture(const HeightmapGeom_Texture& from);

  inline HeightmapGeom_Texture& operator=(const HeightmapGeom_Texture& from) {
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
  static const HeightmapGeom_Texture& default_instance();

  void Swap(HeightmapGeom_Texture* other);

  // implements Message ----------------------------------------------

  inline HeightmapGeom_Texture* New() const { return New(NULL); }

  HeightmapGeom_Texture* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const HeightmapGeom_Texture& from);
  void MergeFrom(const HeightmapGeom_Texture& from);
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
  void InternalSwap(HeightmapGeom_Texture* other);
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

  // required string diffuse = 1;
  bool has_diffuse() const;
  void clear_diffuse();
  static const int kDiffuseFieldNumber = 1;
  const ::std::string& diffuse() const;
  void set_diffuse(const ::std::string& value);
  void set_diffuse(const char* value);
  void set_diffuse(const char* value, size_t size);
  ::std::string* mutable_diffuse();
  ::std::string* release_diffuse();
  void set_allocated_diffuse(::std::string* diffuse);

  // required string normal = 2;
  bool has_normal() const;
  void clear_normal();
  static const int kNormalFieldNumber = 2;
  const ::std::string& normal() const;
  void set_normal(const ::std::string& value);
  void set_normal(const char* value);
  void set_normal(const char* value, size_t size);
  ::std::string* mutable_normal();
  ::std::string* release_normal();
  void set_allocated_normal(::std::string* normal);

  // required double size = 3;
  bool has_size() const;
  void clear_size();
  static const int kSizeFieldNumber = 3;
  double size() const;
  void set_size(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.HeightmapGeom.Texture)
 private:
  inline void set_has_diffuse();
  inline void clear_has_diffuse();
  inline void set_has_normal();
  inline void clear_has_normal();
  inline void set_has_size();
  inline void clear_has_size();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr diffuse_;
  ::google::protobuf::internal::ArenaStringPtr normal_;
  double size_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_heightmapgeom_2eproto();
  friend void protobuf_AssignDesc_heightmapgeom_2eproto();
  friend void protobuf_ShutdownFile_heightmapgeom_2eproto();

  void InitAsDefaultInstance();
  static HeightmapGeom_Texture* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE HeightmapGeom_Blend : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.HeightmapGeom.Blend) */ {
 public:
  HeightmapGeom_Blend();
  virtual ~HeightmapGeom_Blend();

  HeightmapGeom_Blend(const HeightmapGeom_Blend& from);

  inline HeightmapGeom_Blend& operator=(const HeightmapGeom_Blend& from) {
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
  static const HeightmapGeom_Blend& default_instance();

  void Swap(HeightmapGeom_Blend* other);

  // implements Message ----------------------------------------------

  inline HeightmapGeom_Blend* New() const { return New(NULL); }

  HeightmapGeom_Blend* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const HeightmapGeom_Blend& from);
  void MergeFrom(const HeightmapGeom_Blend& from);
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
  void InternalSwap(HeightmapGeom_Blend* other);
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

  // required double min_height = 1;
  bool has_min_height() const;
  void clear_min_height();
  static const int kMinHeightFieldNumber = 1;
  double min_height() const;
  void set_min_height(double value);

  // required double fade_dist = 2;
  bool has_fade_dist() const;
  void clear_fade_dist();
  static const int kFadeDistFieldNumber = 2;
  double fade_dist() const;
  void set_fade_dist(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.HeightmapGeom.Blend)
 private:
  inline void set_has_min_height();
  inline void clear_has_min_height();
  inline void set_has_fade_dist();
  inline void clear_has_fade_dist();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double min_height_;
  double fade_dist_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_heightmapgeom_2eproto();
  friend void protobuf_AssignDesc_heightmapgeom_2eproto();
  friend void protobuf_ShutdownFile_heightmapgeom_2eproto();

  void InitAsDefaultInstance();
  static HeightmapGeom_Blend* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE HeightmapGeom : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.HeightmapGeom) */ {
 public:
  HeightmapGeom();
  virtual ~HeightmapGeom();

  HeightmapGeom(const HeightmapGeom& from);

  inline HeightmapGeom& operator=(const HeightmapGeom& from) {
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
  static const HeightmapGeom& default_instance();

  void Swap(HeightmapGeom* other);

  // implements Message ----------------------------------------------

  inline HeightmapGeom* New() const { return New(NULL); }

  HeightmapGeom* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const HeightmapGeom& from);
  void MergeFrom(const HeightmapGeom& from);
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
  void InternalSwap(HeightmapGeom* other);
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

  typedef HeightmapGeom_Texture Texture;
  typedef HeightmapGeom_Blend Blend;

  // accessors -------------------------------------------------------

  // optional .gazebo.msgs.Image image = 1;
  bool has_image() const;
  void clear_image();
  static const int kImageFieldNumber = 1;
  const ::gazebo::msgs::Image& image() const;
  ::gazebo::msgs::Image* mutable_image();
  ::gazebo::msgs::Image* release_image();
  void set_allocated_image(::gazebo::msgs::Image* image);

  // required .gazebo.msgs.Vector3d size = 2;
  bool has_size() const;
  void clear_size();
  static const int kSizeFieldNumber = 2;
  const ::gazebo::msgs::Vector3d& size() const;
  ::gazebo::msgs::Vector3d* mutable_size();
  ::gazebo::msgs::Vector3d* release_size();
  void set_allocated_size(::gazebo::msgs::Vector3d* size);

  // optional .gazebo.msgs.Vector3d origin = 3;
  bool has_origin() const;
  void clear_origin();
  static const int kOriginFieldNumber = 3;
  const ::gazebo::msgs::Vector3d& origin() const;
  ::gazebo::msgs::Vector3d* mutable_origin();
  ::gazebo::msgs::Vector3d* release_origin();
  void set_allocated_origin(::gazebo::msgs::Vector3d* origin);

  // repeated float heights = 4;
  int heights_size() const;
  void clear_heights();
  static const int kHeightsFieldNumber = 4;
  float heights(int index) const;
  void set_heights(int index, float value);
  void add_heights(float value);
  const ::google::protobuf::RepeatedField< float >&
      heights() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_heights();

  // optional int32 width = 5;
  bool has_width() const;
  void clear_width();
  static const int kWidthFieldNumber = 5;
  ::google::protobuf::int32 width() const;
  void set_width(::google::protobuf::int32 value);

  // optional int32 height = 6;
  bool has_height() const;
  void clear_height();
  static const int kHeightFieldNumber = 6;
  ::google::protobuf::int32 height() const;
  void set_height(::google::protobuf::int32 value);

  // repeated .gazebo.msgs.HeightmapGeom.Texture texture = 7;
  int texture_size() const;
  void clear_texture();
  static const int kTextureFieldNumber = 7;
  const ::gazebo::msgs::HeightmapGeom_Texture& texture(int index) const;
  ::gazebo::msgs::HeightmapGeom_Texture* mutable_texture(int index);
  ::gazebo::msgs::HeightmapGeom_Texture* add_texture();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Texture >*
      mutable_texture();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Texture >&
      texture() const;

  // repeated .gazebo.msgs.HeightmapGeom.Blend blend = 8;
  int blend_size() const;
  void clear_blend();
  static const int kBlendFieldNumber = 8;
  const ::gazebo::msgs::HeightmapGeom_Blend& blend(int index) const;
  ::gazebo::msgs::HeightmapGeom_Blend* mutable_blend(int index);
  ::gazebo::msgs::HeightmapGeom_Blend* add_blend();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Blend >*
      mutable_blend();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Blend >&
      blend() const;

  // optional bool use_terrain_paging = 9;
  bool has_use_terrain_paging() const;
  void clear_use_terrain_paging();
  static const int kUseTerrainPagingFieldNumber = 9;
  bool use_terrain_paging() const;
  void set_use_terrain_paging(bool value);

  // optional string filename = 10;
  bool has_filename() const;
  void clear_filename();
  static const int kFilenameFieldNumber = 10;
  const ::std::string& filename() const;
  void set_filename(const ::std::string& value);
  void set_filename(const char* value);
  void set_filename(const char* value, size_t size);
  ::std::string* mutable_filename();
  ::std::string* release_filename();
  void set_allocated_filename(::std::string* filename);

  // optional uint32 sampling = 11;
  bool has_sampling() const;
  void clear_sampling();
  static const int kSamplingFieldNumber = 11;
  ::google::protobuf::uint32 sampling() const;
  void set_sampling(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.HeightmapGeom)
 private:
  inline void set_has_image();
  inline void clear_has_image();
  inline void set_has_size();
  inline void clear_has_size();
  inline void set_has_origin();
  inline void clear_has_origin();
  inline void set_has_width();
  inline void clear_has_width();
  inline void set_has_height();
  inline void clear_has_height();
  inline void set_has_use_terrain_paging();
  inline void clear_has_use_terrain_paging();
  inline void set_has_filename();
  inline void clear_has_filename();
  inline void set_has_sampling();
  inline void clear_has_sampling();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Image* image_;
  ::gazebo::msgs::Vector3d* size_;
  ::gazebo::msgs::Vector3d* origin_;
  ::google::protobuf::RepeatedField< float > heights_;
  ::google::protobuf::int32 width_;
  ::google::protobuf::int32 height_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Texture > texture_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Blend > blend_;
  ::google::protobuf::internal::ArenaStringPtr filename_;
  bool use_terrain_paging_;
  ::google::protobuf::uint32 sampling_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_heightmapgeom_2eproto();
  friend void protobuf_AssignDesc_heightmapgeom_2eproto();
  friend void protobuf_ShutdownFile_heightmapgeom_2eproto();

  void InitAsDefaultInstance();
  static HeightmapGeom* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// HeightmapGeom_Texture

// required string diffuse = 1;
inline bool HeightmapGeom_Texture::has_diffuse() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void HeightmapGeom_Texture::set_has_diffuse() {
  _has_bits_[0] |= 0x00000001u;
}
inline void HeightmapGeom_Texture::clear_has_diffuse() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void HeightmapGeom_Texture::clear_diffuse() {
  diffuse_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_diffuse();
}
inline const ::std::string& HeightmapGeom_Texture::diffuse() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.Texture.diffuse)
  return diffuse_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void HeightmapGeom_Texture::set_diffuse(const ::std::string& value) {
  set_has_diffuse();
  diffuse_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.Texture.diffuse)
}
inline void HeightmapGeom_Texture::set_diffuse(const char* value) {
  set_has_diffuse();
  diffuse_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.HeightmapGeom.Texture.diffuse)
}
inline void HeightmapGeom_Texture::set_diffuse(const char* value, size_t size) {
  set_has_diffuse();
  diffuse_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.HeightmapGeom.Texture.diffuse)
}
inline ::std::string* HeightmapGeom_Texture::mutable_diffuse() {
  set_has_diffuse();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.Texture.diffuse)
  return diffuse_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* HeightmapGeom_Texture::release_diffuse() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.HeightmapGeom.Texture.diffuse)
  clear_has_diffuse();
  return diffuse_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void HeightmapGeom_Texture::set_allocated_diffuse(::std::string* diffuse) {
  if (diffuse != NULL) {
    set_has_diffuse();
  } else {
    clear_has_diffuse();
  }
  diffuse_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), diffuse);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.HeightmapGeom.Texture.diffuse)
}

// required string normal = 2;
inline bool HeightmapGeom_Texture::has_normal() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void HeightmapGeom_Texture::set_has_normal() {
  _has_bits_[0] |= 0x00000002u;
}
inline void HeightmapGeom_Texture::clear_has_normal() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void HeightmapGeom_Texture::clear_normal() {
  normal_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_normal();
}
inline const ::std::string& HeightmapGeom_Texture::normal() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.Texture.normal)
  return normal_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void HeightmapGeom_Texture::set_normal(const ::std::string& value) {
  set_has_normal();
  normal_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.Texture.normal)
}
inline void HeightmapGeom_Texture::set_normal(const char* value) {
  set_has_normal();
  normal_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.HeightmapGeom.Texture.normal)
}
inline void HeightmapGeom_Texture::set_normal(const char* value, size_t size) {
  set_has_normal();
  normal_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.HeightmapGeom.Texture.normal)
}
inline ::std::string* HeightmapGeom_Texture::mutable_normal() {
  set_has_normal();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.Texture.normal)
  return normal_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* HeightmapGeom_Texture::release_normal() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.HeightmapGeom.Texture.normal)
  clear_has_normal();
  return normal_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void HeightmapGeom_Texture::set_allocated_normal(::std::string* normal) {
  if (normal != NULL) {
    set_has_normal();
  } else {
    clear_has_normal();
  }
  normal_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), normal);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.HeightmapGeom.Texture.normal)
}

// required double size = 3;
inline bool HeightmapGeom_Texture::has_size() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void HeightmapGeom_Texture::set_has_size() {
  _has_bits_[0] |= 0x00000004u;
}
inline void HeightmapGeom_Texture::clear_has_size() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void HeightmapGeom_Texture::clear_size() {
  size_ = 0;
  clear_has_size();
}
inline double HeightmapGeom_Texture::size() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.Texture.size)
  return size_;
}
inline void HeightmapGeom_Texture::set_size(double value) {
  set_has_size();
  size_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.Texture.size)
}

// -------------------------------------------------------------------

// HeightmapGeom_Blend

// required double min_height = 1;
inline bool HeightmapGeom_Blend::has_min_height() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void HeightmapGeom_Blend::set_has_min_height() {
  _has_bits_[0] |= 0x00000001u;
}
inline void HeightmapGeom_Blend::clear_has_min_height() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void HeightmapGeom_Blend::clear_min_height() {
  min_height_ = 0;
  clear_has_min_height();
}
inline double HeightmapGeom_Blend::min_height() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.Blend.min_height)
  return min_height_;
}
inline void HeightmapGeom_Blend::set_min_height(double value) {
  set_has_min_height();
  min_height_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.Blend.min_height)
}

// required double fade_dist = 2;
inline bool HeightmapGeom_Blend::has_fade_dist() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void HeightmapGeom_Blend::set_has_fade_dist() {
  _has_bits_[0] |= 0x00000002u;
}
inline void HeightmapGeom_Blend::clear_has_fade_dist() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void HeightmapGeom_Blend::clear_fade_dist() {
  fade_dist_ = 0;
  clear_has_fade_dist();
}
inline double HeightmapGeom_Blend::fade_dist() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.Blend.fade_dist)
  return fade_dist_;
}
inline void HeightmapGeom_Blend::set_fade_dist(double value) {
  set_has_fade_dist();
  fade_dist_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.Blend.fade_dist)
}

// -------------------------------------------------------------------

// HeightmapGeom

// optional .gazebo.msgs.Image image = 1;
inline bool HeightmapGeom::has_image() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void HeightmapGeom::set_has_image() {
  _has_bits_[0] |= 0x00000001u;
}
inline void HeightmapGeom::clear_has_image() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void HeightmapGeom::clear_image() {
  if (image_ != NULL) image_->::gazebo::msgs::Image::Clear();
  clear_has_image();
}
inline const ::gazebo::msgs::Image& HeightmapGeom::image() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.image)
  return image_ != NULL ? *image_ : *default_instance_->image_;
}
inline ::gazebo::msgs::Image* HeightmapGeom::mutable_image() {
  set_has_image();
  if (image_ == NULL) {
    image_ = new ::gazebo::msgs::Image;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.image)
  return image_;
}
inline ::gazebo::msgs::Image* HeightmapGeom::release_image() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.HeightmapGeom.image)
  clear_has_image();
  ::gazebo::msgs::Image* temp = image_;
  image_ = NULL;
  return temp;
}
inline void HeightmapGeom::set_allocated_image(::gazebo::msgs::Image* image) {
  delete image_;
  image_ = image;
  if (image) {
    set_has_image();
  } else {
    clear_has_image();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.HeightmapGeom.image)
}

// required .gazebo.msgs.Vector3d size = 2;
inline bool HeightmapGeom::has_size() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void HeightmapGeom::set_has_size() {
  _has_bits_[0] |= 0x00000002u;
}
inline void HeightmapGeom::clear_has_size() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void HeightmapGeom::clear_size() {
  if (size_ != NULL) size_->::gazebo::msgs::Vector3d::Clear();
  clear_has_size();
}
inline const ::gazebo::msgs::Vector3d& HeightmapGeom::size() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.size)
  return size_ != NULL ? *size_ : *default_instance_->size_;
}
inline ::gazebo::msgs::Vector3d* HeightmapGeom::mutable_size() {
  set_has_size();
  if (size_ == NULL) {
    size_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.size)
  return size_;
}
inline ::gazebo::msgs::Vector3d* HeightmapGeom::release_size() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.HeightmapGeom.size)
  clear_has_size();
  ::gazebo::msgs::Vector3d* temp = size_;
  size_ = NULL;
  return temp;
}
inline void HeightmapGeom::set_allocated_size(::gazebo::msgs::Vector3d* size) {
  delete size_;
  size_ = size;
  if (size) {
    set_has_size();
  } else {
    clear_has_size();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.HeightmapGeom.size)
}

// optional .gazebo.msgs.Vector3d origin = 3;
inline bool HeightmapGeom::has_origin() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void HeightmapGeom::set_has_origin() {
  _has_bits_[0] |= 0x00000004u;
}
inline void HeightmapGeom::clear_has_origin() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void HeightmapGeom::clear_origin() {
  if (origin_ != NULL) origin_->::gazebo::msgs::Vector3d::Clear();
  clear_has_origin();
}
inline const ::gazebo::msgs::Vector3d& HeightmapGeom::origin() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.origin)
  return origin_ != NULL ? *origin_ : *default_instance_->origin_;
}
inline ::gazebo::msgs::Vector3d* HeightmapGeom::mutable_origin() {
  set_has_origin();
  if (origin_ == NULL) {
    origin_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.origin)
  return origin_;
}
inline ::gazebo::msgs::Vector3d* HeightmapGeom::release_origin() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.HeightmapGeom.origin)
  clear_has_origin();
  ::gazebo::msgs::Vector3d* temp = origin_;
  origin_ = NULL;
  return temp;
}
inline void HeightmapGeom::set_allocated_origin(::gazebo::msgs::Vector3d* origin) {
  delete origin_;
  origin_ = origin;
  if (origin) {
    set_has_origin();
  } else {
    clear_has_origin();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.HeightmapGeom.origin)
}

// repeated float heights = 4;
inline int HeightmapGeom::heights_size() const {
  return heights_.size();
}
inline void HeightmapGeom::clear_heights() {
  heights_.Clear();
}
inline float HeightmapGeom::heights(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.heights)
  return heights_.Get(index);
}
inline void HeightmapGeom::set_heights(int index, float value) {
  heights_.Set(index, value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.heights)
}
inline void HeightmapGeom::add_heights(float value) {
  heights_.Add(value);
  // @@protoc_insertion_point(field_add:gazebo.msgs.HeightmapGeom.heights)
}
inline const ::google::protobuf::RepeatedField< float >&
HeightmapGeom::heights() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.HeightmapGeom.heights)
  return heights_;
}
inline ::google::protobuf::RepeatedField< float >*
HeightmapGeom::mutable_heights() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.HeightmapGeom.heights)
  return &heights_;
}

// optional int32 width = 5;
inline bool HeightmapGeom::has_width() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void HeightmapGeom::set_has_width() {
  _has_bits_[0] |= 0x00000010u;
}
inline void HeightmapGeom::clear_has_width() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void HeightmapGeom::clear_width() {
  width_ = 0;
  clear_has_width();
}
inline ::google::protobuf::int32 HeightmapGeom::width() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.width)
  return width_;
}
inline void HeightmapGeom::set_width(::google::protobuf::int32 value) {
  set_has_width();
  width_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.width)
}

// optional int32 height = 6;
inline bool HeightmapGeom::has_height() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void HeightmapGeom::set_has_height() {
  _has_bits_[0] |= 0x00000020u;
}
inline void HeightmapGeom::clear_has_height() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void HeightmapGeom::clear_height() {
  height_ = 0;
  clear_has_height();
}
inline ::google::protobuf::int32 HeightmapGeom::height() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.height)
  return height_;
}
inline void HeightmapGeom::set_height(::google::protobuf::int32 value) {
  set_has_height();
  height_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.height)
}

// repeated .gazebo.msgs.HeightmapGeom.Texture texture = 7;
inline int HeightmapGeom::texture_size() const {
  return texture_.size();
}
inline void HeightmapGeom::clear_texture() {
  texture_.Clear();
}
inline const ::gazebo::msgs::HeightmapGeom_Texture& HeightmapGeom::texture(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.texture)
  return texture_.Get(index);
}
inline ::gazebo::msgs::HeightmapGeom_Texture* HeightmapGeom::mutable_texture(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.texture)
  return texture_.Mutable(index);
}
inline ::gazebo::msgs::HeightmapGeom_Texture* HeightmapGeom::add_texture() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.HeightmapGeom.texture)
  return texture_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Texture >*
HeightmapGeom::mutable_texture() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.HeightmapGeom.texture)
  return &texture_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Texture >&
HeightmapGeom::texture() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.HeightmapGeom.texture)
  return texture_;
}

// repeated .gazebo.msgs.HeightmapGeom.Blend blend = 8;
inline int HeightmapGeom::blend_size() const {
  return blend_.size();
}
inline void HeightmapGeom::clear_blend() {
  blend_.Clear();
}
inline const ::gazebo::msgs::HeightmapGeom_Blend& HeightmapGeom::blend(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.blend)
  return blend_.Get(index);
}
inline ::gazebo::msgs::HeightmapGeom_Blend* HeightmapGeom::mutable_blend(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.blend)
  return blend_.Mutable(index);
}
inline ::gazebo::msgs::HeightmapGeom_Blend* HeightmapGeom::add_blend() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.HeightmapGeom.blend)
  return blend_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Blend >*
HeightmapGeom::mutable_blend() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.HeightmapGeom.blend)
  return &blend_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::HeightmapGeom_Blend >&
HeightmapGeom::blend() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.HeightmapGeom.blend)
  return blend_;
}

// optional bool use_terrain_paging = 9;
inline bool HeightmapGeom::has_use_terrain_paging() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void HeightmapGeom::set_has_use_terrain_paging() {
  _has_bits_[0] |= 0x00000100u;
}
inline void HeightmapGeom::clear_has_use_terrain_paging() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void HeightmapGeom::clear_use_terrain_paging() {
  use_terrain_paging_ = false;
  clear_has_use_terrain_paging();
}
inline bool HeightmapGeom::use_terrain_paging() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.use_terrain_paging)
  return use_terrain_paging_;
}
inline void HeightmapGeom::set_use_terrain_paging(bool value) {
  set_has_use_terrain_paging();
  use_terrain_paging_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.use_terrain_paging)
}

// optional string filename = 10;
inline bool HeightmapGeom::has_filename() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void HeightmapGeom::set_has_filename() {
  _has_bits_[0] |= 0x00000200u;
}
inline void HeightmapGeom::clear_has_filename() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void HeightmapGeom::clear_filename() {
  filename_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_filename();
}
inline const ::std::string& HeightmapGeom::filename() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.filename)
  return filename_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void HeightmapGeom::set_filename(const ::std::string& value) {
  set_has_filename();
  filename_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.filename)
}
inline void HeightmapGeom::set_filename(const char* value) {
  set_has_filename();
  filename_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.HeightmapGeom.filename)
}
inline void HeightmapGeom::set_filename(const char* value, size_t size) {
  set_has_filename();
  filename_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.HeightmapGeom.filename)
}
inline ::std::string* HeightmapGeom::mutable_filename() {
  set_has_filename();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.HeightmapGeom.filename)
  return filename_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* HeightmapGeom::release_filename() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.HeightmapGeom.filename)
  clear_has_filename();
  return filename_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void HeightmapGeom::set_allocated_filename(::std::string* filename) {
  if (filename != NULL) {
    set_has_filename();
  } else {
    clear_has_filename();
  }
  filename_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), filename);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.HeightmapGeom.filename)
}

// optional uint32 sampling = 11;
inline bool HeightmapGeom::has_sampling() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void HeightmapGeom::set_has_sampling() {
  _has_bits_[0] |= 0x00000400u;
}
inline void HeightmapGeom::clear_has_sampling() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void HeightmapGeom::clear_sampling() {
  sampling_ = 0u;
  clear_has_sampling();
}
inline ::google::protobuf::uint32 HeightmapGeom::sampling() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.HeightmapGeom.sampling)
  return sampling_;
}
inline void HeightmapGeom::set_sampling(::google::protobuf::uint32 value) {
  set_has_sampling();
  sampling_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.HeightmapGeom.sampling)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------


typedef boost::shared_ptr<gazebo::msgs::HeightmapGeom> HeightmapGeomPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::HeightmapGeom const> ConstHeightmapGeomPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_heightmapgeom_2eproto__INCLUDED