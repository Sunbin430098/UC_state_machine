// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: logical_camera_image.proto

#ifndef PROTOBUF_logical_5fcamera_5fimage_2eproto__INCLUDED
#define PROTOBUF_logical_5fcamera_5fimage_2eproto__INCLUDED

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
#include "pose.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_logical_5fcamera_5fimage_2eproto();
void protobuf_AssignDesc_logical_5fcamera_5fimage_2eproto();
void protobuf_ShutdownFile_logical_5fcamera_5fimage_2eproto();

class LogicalCameraImage;
class LogicalCameraImage_Model;

// ===================================================================

class GZ_MSGS_VISIBLE LogicalCameraImage_Model : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.LogicalCameraImage.Model) */ {
 public:
  LogicalCameraImage_Model();
  virtual ~LogicalCameraImage_Model();

  LogicalCameraImage_Model(const LogicalCameraImage_Model& from);

  inline LogicalCameraImage_Model& operator=(const LogicalCameraImage_Model& from) {
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
  static const LogicalCameraImage_Model& default_instance();

  void Swap(LogicalCameraImage_Model* other);

  // implements Message ----------------------------------------------

  inline LogicalCameraImage_Model* New() const { return New(NULL); }

  LogicalCameraImage_Model* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const LogicalCameraImage_Model& from);
  void MergeFrom(const LogicalCameraImage_Model& from);
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
  void InternalSwap(LogicalCameraImage_Model* other);
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

  // required .gazebo.msgs.Pose pose = 2;
  bool has_pose() const;
  void clear_pose();
  static const int kPoseFieldNumber = 2;
  const ::gazebo::msgs::Pose& pose() const;
  ::gazebo::msgs::Pose* mutable_pose();
  ::gazebo::msgs::Pose* release_pose();
  void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.LogicalCameraImage.Model)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_pose();
  inline void clear_has_pose();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  ::gazebo::msgs::Pose* pose_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_logical_5fcamera_5fimage_2eproto();
  friend void protobuf_AssignDesc_logical_5fcamera_5fimage_2eproto();
  friend void protobuf_ShutdownFile_logical_5fcamera_5fimage_2eproto();

  void InitAsDefaultInstance();
  static LogicalCameraImage_Model* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE LogicalCameraImage : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.LogicalCameraImage) */ {
 public:
  LogicalCameraImage();
  virtual ~LogicalCameraImage();

  LogicalCameraImage(const LogicalCameraImage& from);

  inline LogicalCameraImage& operator=(const LogicalCameraImage& from) {
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
  static const LogicalCameraImage& default_instance();

  void Swap(LogicalCameraImage* other);

  // implements Message ----------------------------------------------

  inline LogicalCameraImage* New() const { return New(NULL); }

  LogicalCameraImage* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const LogicalCameraImage& from);
  void MergeFrom(const LogicalCameraImage& from);
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
  void InternalSwap(LogicalCameraImage* other);
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

  typedef LogicalCameraImage_Model Model;

  // accessors -------------------------------------------------------

  // required .gazebo.msgs.Pose pose = 1;
  bool has_pose() const;
  void clear_pose();
  static const int kPoseFieldNumber = 1;
  const ::gazebo::msgs::Pose& pose() const;
  ::gazebo::msgs::Pose* mutable_pose();
  ::gazebo::msgs::Pose* release_pose();
  void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // repeated .gazebo.msgs.LogicalCameraImage.Model model = 2;
  int model_size() const;
  void clear_model();
  static const int kModelFieldNumber = 2;
  const ::gazebo::msgs::LogicalCameraImage_Model& model(int index) const;
  ::gazebo::msgs::LogicalCameraImage_Model* mutable_model(int index);
  ::gazebo::msgs::LogicalCameraImage_Model* add_model();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::LogicalCameraImage_Model >*
      mutable_model();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::LogicalCameraImage_Model >&
      model() const;

  // @@protoc_insertion_point(class_scope:gazebo.msgs.LogicalCameraImage)
 private:
  inline void set_has_pose();
  inline void clear_has_pose();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::Pose* pose_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::LogicalCameraImage_Model > model_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_logical_5fcamera_5fimage_2eproto();
  friend void protobuf_AssignDesc_logical_5fcamera_5fimage_2eproto();
  friend void protobuf_ShutdownFile_logical_5fcamera_5fimage_2eproto();

  void InitAsDefaultInstance();
  static LogicalCameraImage* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// LogicalCameraImage_Model

// required string name = 1;
inline bool LogicalCameraImage_Model::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LogicalCameraImage_Model::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void LogicalCameraImage_Model::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void LogicalCameraImage_Model::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& LogicalCameraImage_Model::name() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LogicalCameraImage.Model.name)
  return name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void LogicalCameraImage_Model::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.LogicalCameraImage.Model.name)
}
inline void LogicalCameraImage_Model::set_name(const char* value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.LogicalCameraImage.Model.name)
}
inline void LogicalCameraImage_Model::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.LogicalCameraImage.Model.name)
}
inline ::std::string* LogicalCameraImage_Model::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.LogicalCameraImage.Model.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* LogicalCameraImage_Model::release_name() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.LogicalCameraImage.Model.name)
  clear_has_name();
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void LogicalCameraImage_Model::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.LogicalCameraImage.Model.name)
}

// required .gazebo.msgs.Pose pose = 2;
inline bool LogicalCameraImage_Model::has_pose() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LogicalCameraImage_Model::set_has_pose() {
  _has_bits_[0] |= 0x00000002u;
}
inline void LogicalCameraImage_Model::clear_has_pose() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void LogicalCameraImage_Model::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& LogicalCameraImage_Model::pose() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LogicalCameraImage.Model.pose)
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* LogicalCameraImage_Model::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) {
    pose_ = new ::gazebo::msgs::Pose;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.LogicalCameraImage.Model.pose)
  return pose_;
}
inline ::gazebo::msgs::Pose* LogicalCameraImage_Model::release_pose() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.LogicalCameraImage.Model.pose)
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void LogicalCameraImage_Model::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.LogicalCameraImage.Model.pose)
}

// -------------------------------------------------------------------

// LogicalCameraImage

// required .gazebo.msgs.Pose pose = 1;
inline bool LogicalCameraImage::has_pose() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LogicalCameraImage::set_has_pose() {
  _has_bits_[0] |= 0x00000001u;
}
inline void LogicalCameraImage::clear_has_pose() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void LogicalCameraImage::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& LogicalCameraImage::pose() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LogicalCameraImage.pose)
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* LogicalCameraImage::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) {
    pose_ = new ::gazebo::msgs::Pose;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.LogicalCameraImage.pose)
  return pose_;
}
inline ::gazebo::msgs::Pose* LogicalCameraImage::release_pose() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.LogicalCameraImage.pose)
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void LogicalCameraImage::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.LogicalCameraImage.pose)
}

// repeated .gazebo.msgs.LogicalCameraImage.Model model = 2;
inline int LogicalCameraImage::model_size() const {
  return model_.size();
}
inline void LogicalCameraImage::clear_model() {
  model_.Clear();
}
inline const ::gazebo::msgs::LogicalCameraImage_Model& LogicalCameraImage::model(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LogicalCameraImage.model)
  return model_.Get(index);
}
inline ::gazebo::msgs::LogicalCameraImage_Model* LogicalCameraImage::mutable_model(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.LogicalCameraImage.model)
  return model_.Mutable(index);
}
inline ::gazebo::msgs::LogicalCameraImage_Model* LogicalCameraImage::add_model() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.LogicalCameraImage.model)
  return model_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::LogicalCameraImage_Model >*
LogicalCameraImage::mutable_model() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.LogicalCameraImage.model)
  return &model_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::LogicalCameraImage_Model >&
LogicalCameraImage::model() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.LogicalCameraImage.model)
  return model_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


typedef boost::shared_ptr<gazebo::msgs::LogicalCameraImage> LogicalCameraImagePtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::LogicalCameraImage const> ConstLogicalCameraImagePtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_logical_5fcamera_5fimage_2eproto__INCLUDED
