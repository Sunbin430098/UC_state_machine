// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pointcloud.proto

#ifndef PROTOBUF_pointcloud_2eproto__INCLUDED
#define PROTOBUF_pointcloud_2eproto__INCLUDED

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
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_pointcloud_2eproto();
void protobuf_AssignDesc_pointcloud_2eproto();
void protobuf_ShutdownFile_pointcloud_2eproto();

class PointCloud;

// ===================================================================

class GZ_MSGS_VISIBLE PointCloud : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.PointCloud) */ {
 public:
  PointCloud();
  virtual ~PointCloud();

  PointCloud(const PointCloud& from);

  inline PointCloud& operator=(const PointCloud& from) {
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
  static const PointCloud& default_instance();

  void Swap(PointCloud* other);

  // implements Message ----------------------------------------------

  inline PointCloud* New() const { return New(NULL); }

  PointCloud* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const PointCloud& from);
  void MergeFrom(const PointCloud& from);
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
  void InternalSwap(PointCloud* other);
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

  // repeated .gazebo.msgs.Vector3d points = 1;
  int points_size() const;
  void clear_points();
  static const int kPointsFieldNumber = 1;
  const ::gazebo::msgs::Vector3d& points(int index) const;
  ::gazebo::msgs::Vector3d* mutable_points(int index);
  ::gazebo::msgs::Vector3d* add_points();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
      mutable_points();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
      points() const;

  // @@protoc_insertion_point(class_scope:gazebo.msgs.PointCloud)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d > points_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_pointcloud_2eproto();
  friend void protobuf_AssignDesc_pointcloud_2eproto();
  friend void protobuf_ShutdownFile_pointcloud_2eproto();

  void InitAsDefaultInstance();
  static PointCloud* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// PointCloud

// repeated .gazebo.msgs.Vector3d points = 1;
inline int PointCloud::points_size() const {
  return points_.size();
}
inline void PointCloud::clear_points() {
  points_.Clear();
}
inline const ::gazebo::msgs::Vector3d& PointCloud::points(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.PointCloud.points)
  return points_.Get(index);
}
inline ::gazebo::msgs::Vector3d* PointCloud::mutable_points(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.PointCloud.points)
  return points_.Mutable(index);
}
inline ::gazebo::msgs::Vector3d* PointCloud::add_points() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.PointCloud.points)
  return points_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
PointCloud::mutable_points() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.PointCloud.points)
  return &points_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
PointCloud::points() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.PointCloud.points)
  return points_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::PointCloud> PointCloudPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::PointCloud const> ConstPointCloudPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_pointcloud_2eproto__INCLUDED
