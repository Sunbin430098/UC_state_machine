// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: road.proto

#ifndef PROTOBUF_road_2eproto__INCLUDED
#define PROTOBUF_road_2eproto__INCLUDED

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
#include "material.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_road_2eproto();
void protobuf_AssignDesc_road_2eproto();
void protobuf_ShutdownFile_road_2eproto();

class Road;

// ===================================================================

class GZ_MSGS_VISIBLE Road : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Road) */ {
 public:
  Road();
  virtual ~Road();

  Road(const Road& from);

  inline Road& operator=(const Road& from) {
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
  static const Road& default_instance();

  void Swap(Road* other);

  // implements Message ----------------------------------------------

  inline Road* New() const { return New(NULL); }

  Road* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Road& from);
  void MergeFrom(const Road& from);
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
  void InternalSwap(Road* other);
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

  // required string name = 1;
  bool has_name() const;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // required double width = 2;
  bool has_width() const;
  void clear_width();
  static const int kWidthFieldNumber = 2;
  double width() const;
  void set_width(double value);

  // repeated .gazebo.msgs.Vector3d point = 3;
  int point_size() const;
  void clear_point();
  static const int kPointFieldNumber = 3;
  const ::gazebo::msgs::Vector3d& point(int index) const;
  ::gazebo::msgs::Vector3d* mutable_point(int index);
  ::gazebo::msgs::Vector3d* add_point();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
      mutable_point();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
      point() const;

  // optional .gazebo.msgs.Material material = 4;
  bool has_material() const;
  void clear_material();
  static const int kMaterialFieldNumber = 4;
  const ::gazebo::msgs::Material& material() const;
  ::gazebo::msgs::Material* mutable_material();
  ::gazebo::msgs::Material* release_material();
  void set_allocated_material(::gazebo::msgs::Material* material);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Road)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_width();
  inline void clear_has_width();
  inline void set_has_material();
  inline void clear_has_material();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  double width_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d > point_;
  ::gazebo::msgs::Material* material_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_road_2eproto();
  friend void protobuf_AssignDesc_road_2eproto();
  friend void protobuf_ShutdownFile_road_2eproto();

  void InitAsDefaultInstance();
  static Road* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Road

// required string name = 1;
inline bool Road::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Road::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Road::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Road::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& Road::name() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Road.name)
  return name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Road::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Road.name)
}
inline void Road::set_name(const char* value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Road.name)
}
inline void Road::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Road.name)
}
inline ::std::string* Road::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Road.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Road::release_name() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Road.name)
  clear_has_name();
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Road::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Road.name)
}

// required double width = 2;
inline bool Road::has_width() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Road::set_has_width() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Road::clear_has_width() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Road::clear_width() {
  width_ = 0;
  clear_has_width();
}
inline double Road::width() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Road.width)
  return width_;
}
inline void Road::set_width(double value) {
  set_has_width();
  width_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Road.width)
}

// repeated .gazebo.msgs.Vector3d point = 3;
inline int Road::point_size() const {
  return point_.size();
}
inline void Road::clear_point() {
  point_.Clear();
}
inline const ::gazebo::msgs::Vector3d& Road::point(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Road.point)
  return point_.Get(index);
}
inline ::gazebo::msgs::Vector3d* Road::mutable_point(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Road.point)
  return point_.Mutable(index);
}
inline ::gazebo::msgs::Vector3d* Road::add_point() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.Road.point)
  return point_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
Road::mutable_point() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Road.point)
  return &point_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
Road::point() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Road.point)
  return point_;
}

// optional .gazebo.msgs.Material material = 4;
inline bool Road::has_material() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Road::set_has_material() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Road::clear_has_material() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Road::clear_material() {
  if (material_ != NULL) material_->::gazebo::msgs::Material::Clear();
  clear_has_material();
}
inline const ::gazebo::msgs::Material& Road::material() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Road.material)
  return material_ != NULL ? *material_ : *default_instance_->material_;
}
inline ::gazebo::msgs::Material* Road::mutable_material() {
  set_has_material();
  if (material_ == NULL) {
    material_ = new ::gazebo::msgs::Material;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Road.material)
  return material_;
}
inline ::gazebo::msgs::Material* Road::release_material() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Road.material)
  clear_has_material();
  ::gazebo::msgs::Material* temp = material_;
  material_ = NULL;
  return temp;
}
inline void Road::set_allocated_material(::gazebo::msgs::Material* material) {
  delete material_;
  material_ = material;
  if (material) {
    set_has_material();
  } else {
    clear_has_material();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Road.material)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::Road> RoadPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::Road const> ConstRoadPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_road_2eproto__INCLUDED
