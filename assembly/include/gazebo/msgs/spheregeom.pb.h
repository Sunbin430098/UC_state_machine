// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: spheregeom.proto

#ifndef PROTOBUF_spheregeom_2eproto__INCLUDED
#define PROTOBUF_spheregeom_2eproto__INCLUDED

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
void GZ_MSGS_VISIBLE protobuf_AddDesc_spheregeom_2eproto();
void protobuf_AssignDesc_spheregeom_2eproto();
void protobuf_ShutdownFile_spheregeom_2eproto();

class SphereGeom;

// ===================================================================

class GZ_MSGS_VISIBLE SphereGeom : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.SphereGeom) */ {
 public:
  SphereGeom();
  virtual ~SphereGeom();

  SphereGeom(const SphereGeom& from);

  inline SphereGeom& operator=(const SphereGeom& from) {
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
  static const SphereGeom& default_instance();

  void Swap(SphereGeom* other);

  // implements Message ----------------------------------------------

  inline SphereGeom* New() const { return New(NULL); }

  SphereGeom* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const SphereGeom& from);
  void MergeFrom(const SphereGeom& from);
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
  void InternalSwap(SphereGeom* other);
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

  // required double radius = 1;
  bool has_radius() const;
  void clear_radius();
  static const int kRadiusFieldNumber = 1;
  double radius() const;
  void set_radius(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.SphereGeom)
 private:
  inline void set_has_radius();
  inline void clear_has_radius();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double radius_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_spheregeom_2eproto();
  friend void protobuf_AssignDesc_spheregeom_2eproto();
  friend void protobuf_ShutdownFile_spheregeom_2eproto();

  void InitAsDefaultInstance();
  static SphereGeom* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// SphereGeom

// required double radius = 1;
inline bool SphereGeom::has_radius() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SphereGeom::set_has_radius() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SphereGeom::clear_has_radius() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SphereGeom::clear_radius() {
  radius_ = 0;
  clear_has_radius();
}
inline double SphereGeom::radius() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SphereGeom.radius)
  return radius_;
}
inline void SphereGeom::set_radius(double value) {
  set_has_radius();
  radius_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.SphereGeom.radius)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::SphereGeom> SphereGeomPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::SphereGeom const> ConstSphereGeomPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_spheregeom_2eproto__INCLUDED