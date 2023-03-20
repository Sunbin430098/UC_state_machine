// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: planegeom.proto

#ifndef PROTOBUF_planegeom_2eproto__INCLUDED
#define PROTOBUF_planegeom_2eproto__INCLUDED

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
#include "vector2d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_planegeom_2eproto();
void protobuf_AssignDesc_planegeom_2eproto();
void protobuf_ShutdownFile_planegeom_2eproto();

class PlaneGeom;

// ===================================================================

class GZ_MSGS_VISIBLE PlaneGeom : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.PlaneGeom) */ {
 public:
  PlaneGeom();
  virtual ~PlaneGeom();

  PlaneGeom(const PlaneGeom& from);

  inline PlaneGeom& operator=(const PlaneGeom& from) {
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
  static const PlaneGeom& default_instance();

  void Swap(PlaneGeom* other);

  // implements Message ----------------------------------------------

  inline PlaneGeom* New() const { return New(NULL); }

  PlaneGeom* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const PlaneGeom& from);
  void MergeFrom(const PlaneGeom& from);
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
  void InternalSwap(PlaneGeom* other);
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

  // required .gazebo.msgs.Vector3d normal = 1;
  bool has_normal() const;
  void clear_normal();
  static const int kNormalFieldNumber = 1;
  const ::gazebo::msgs::Vector3d& normal() const;
  ::gazebo::msgs::Vector3d* mutable_normal();
  ::gazebo::msgs::Vector3d* release_normal();
  void set_allocated_normal(::gazebo::msgs::Vector3d* normal);

  // required .gazebo.msgs.Vector2d size = 2;
  bool has_size() const;
  void clear_size();
  static const int kSizeFieldNumber = 2;
  const ::gazebo::msgs::Vector2d& size() const;
  ::gazebo::msgs::Vector2d* mutable_size();
  ::gazebo::msgs::Vector2d* release_size();
  void set_allocated_size(::gazebo::msgs::Vector2d* size);

  // optional double d = 3 [default = 0];
  bool has_d() const;
  void clear_d();
  static const int kDFieldNumber = 3;
  double d() const;
  void set_d(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.PlaneGeom)
 private:
  inline void set_has_normal();
  inline void clear_has_normal();
  inline void set_has_size();
  inline void clear_has_size();
  inline void set_has_d();
  inline void clear_has_d();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Vector3d* normal_;
  ::gazebo::msgs::Vector2d* size_;
  double d_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_planegeom_2eproto();
  friend void protobuf_AssignDesc_planegeom_2eproto();
  friend void protobuf_ShutdownFile_planegeom_2eproto();

  void InitAsDefaultInstance();
  static PlaneGeom* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// PlaneGeom

// required .gazebo.msgs.Vector3d normal = 1;
inline bool PlaneGeom::has_normal() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PlaneGeom::set_has_normal() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PlaneGeom::clear_has_normal() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PlaneGeom::clear_normal() {
  if (normal_ != NULL) normal_->::gazebo::msgs::Vector3d::Clear();
  clear_has_normal();
}
inline const ::gazebo::msgs::Vector3d& PlaneGeom::normal() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.PlaneGeom.normal)
  return normal_ != NULL ? *normal_ : *default_instance_->normal_;
}
inline ::gazebo::msgs::Vector3d* PlaneGeom::mutable_normal() {
  set_has_normal();
  if (normal_ == NULL) {
    normal_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.PlaneGeom.normal)
  return normal_;
}
inline ::gazebo::msgs::Vector3d* PlaneGeom::release_normal() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.PlaneGeom.normal)
  clear_has_normal();
  ::gazebo::msgs::Vector3d* temp = normal_;
  normal_ = NULL;
  return temp;
}
inline void PlaneGeom::set_allocated_normal(::gazebo::msgs::Vector3d* normal) {
  delete normal_;
  normal_ = normal;
  if (normal) {
    set_has_normal();
  } else {
    clear_has_normal();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.PlaneGeom.normal)
}

// required .gazebo.msgs.Vector2d size = 2;
inline bool PlaneGeom::has_size() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PlaneGeom::set_has_size() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PlaneGeom::clear_has_size() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void PlaneGeom::clear_size() {
  if (size_ != NULL) size_->::gazebo::msgs::Vector2d::Clear();
  clear_has_size();
}
inline const ::gazebo::msgs::Vector2d& PlaneGeom::size() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.PlaneGeom.size)
  return size_ != NULL ? *size_ : *default_instance_->size_;
}
inline ::gazebo::msgs::Vector2d* PlaneGeom::mutable_size() {
  set_has_size();
  if (size_ == NULL) {
    size_ = new ::gazebo::msgs::Vector2d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.PlaneGeom.size)
  return size_;
}
inline ::gazebo::msgs::Vector2d* PlaneGeom::release_size() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.PlaneGeom.size)
  clear_has_size();
  ::gazebo::msgs::Vector2d* temp = size_;
  size_ = NULL;
  return temp;
}
inline void PlaneGeom::set_allocated_size(::gazebo::msgs::Vector2d* size) {
  delete size_;
  size_ = size;
  if (size) {
    set_has_size();
  } else {
    clear_has_size();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.PlaneGeom.size)
}

// optional double d = 3 [default = 0];
inline bool PlaneGeom::has_d() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void PlaneGeom::set_has_d() {
  _has_bits_[0] |= 0x00000004u;
}
inline void PlaneGeom::clear_has_d() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void PlaneGeom::clear_d() {
  d_ = 0;
  clear_has_d();
}
inline double PlaneGeom::d() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.PlaneGeom.d)
  return d_;
}
inline void PlaneGeom::set_d(double value) {
  set_has_d();
  d_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.PlaneGeom.d)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::PlaneGeom> PlaneGeomPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::PlaneGeom const> ConstPlaneGeomPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_planegeom_2eproto__INCLUDED
