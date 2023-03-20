// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gui.proto

#ifndef PROTOBUF_gui_2eproto__INCLUDED
#define PROTOBUF_gui_2eproto__INCLUDED

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
#include "gui_camera.pb.h"
#include "plugin.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_gui_2eproto();
void protobuf_AssignDesc_gui_2eproto();
void protobuf_ShutdownFile_gui_2eproto();

class GUI;

// ===================================================================

class GZ_MSGS_VISIBLE GUI : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.GUI) */ {
 public:
  GUI();
  virtual ~GUI();

  GUI(const GUI& from);

  inline GUI& operator=(const GUI& from) {
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
  static const GUI& default_instance();

  void Swap(GUI* other);

  // implements Message ----------------------------------------------

  inline GUI* New() const { return New(NULL); }

  GUI* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const GUI& from);
  void MergeFrom(const GUI& from);
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
  void InternalSwap(GUI* other);
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

  // optional bool fullscreen = 1;
  bool has_fullscreen() const;
  void clear_fullscreen();
  static const int kFullscreenFieldNumber = 1;
  bool fullscreen() const;
  void set_fullscreen(bool value);

  // optional .gazebo.msgs.GUICamera camera = 2;
  bool has_camera() const;
  void clear_camera();
  static const int kCameraFieldNumber = 2;
  const ::gazebo::msgs::GUICamera& camera() const;
  ::gazebo::msgs::GUICamera* mutable_camera();
  ::gazebo::msgs::GUICamera* release_camera();
  void set_allocated_camera(::gazebo::msgs::GUICamera* camera);

  // repeated .gazebo.msgs.Plugin plugin = 3;
  int plugin_size() const;
  void clear_plugin();
  static const int kPluginFieldNumber = 3;
  const ::gazebo::msgs::Plugin& plugin(int index) const;
  ::gazebo::msgs::Plugin* mutable_plugin(int index);
  ::gazebo::msgs::Plugin* add_plugin();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >*
      mutable_plugin();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >&
      plugin() const;

  // @@protoc_insertion_point(class_scope:gazebo.msgs.GUI)
 private:
  inline void set_has_fullscreen();
  inline void clear_has_fullscreen();
  inline void set_has_camera();
  inline void clear_has_camera();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::gazebo::msgs::GUICamera* camera_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin > plugin_;
  bool fullscreen_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_gui_2eproto();
  friend void protobuf_AssignDesc_gui_2eproto();
  friend void protobuf_ShutdownFile_gui_2eproto();

  void InitAsDefaultInstance();
  static GUI* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// GUI

// optional bool fullscreen = 1;
inline bool GUI::has_fullscreen() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void GUI::set_has_fullscreen() {
  _has_bits_[0] |= 0x00000001u;
}
inline void GUI::clear_has_fullscreen() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void GUI::clear_fullscreen() {
  fullscreen_ = false;
  clear_has_fullscreen();
}
inline bool GUI::fullscreen() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.GUI.fullscreen)
  return fullscreen_;
}
inline void GUI::set_fullscreen(bool value) {
  set_has_fullscreen();
  fullscreen_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.GUI.fullscreen)
}

// optional .gazebo.msgs.GUICamera camera = 2;
inline bool GUI::has_camera() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void GUI::set_has_camera() {
  _has_bits_[0] |= 0x00000002u;
}
inline void GUI::clear_has_camera() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void GUI::clear_camera() {
  if (camera_ != NULL) camera_->::gazebo::msgs::GUICamera::Clear();
  clear_has_camera();
}
inline const ::gazebo::msgs::GUICamera& GUI::camera() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.GUI.camera)
  return camera_ != NULL ? *camera_ : *default_instance_->camera_;
}
inline ::gazebo::msgs::GUICamera* GUI::mutable_camera() {
  set_has_camera();
  if (camera_ == NULL) {
    camera_ = new ::gazebo::msgs::GUICamera;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.GUI.camera)
  return camera_;
}
inline ::gazebo::msgs::GUICamera* GUI::release_camera() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.GUI.camera)
  clear_has_camera();
  ::gazebo::msgs::GUICamera* temp = camera_;
  camera_ = NULL;
  return temp;
}
inline void GUI::set_allocated_camera(::gazebo::msgs::GUICamera* camera) {
  delete camera_;
  camera_ = camera;
  if (camera) {
    set_has_camera();
  } else {
    clear_has_camera();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.GUI.camera)
}

// repeated .gazebo.msgs.Plugin plugin = 3;
inline int GUI::plugin_size() const {
  return plugin_.size();
}
inline void GUI::clear_plugin() {
  plugin_.Clear();
}
inline const ::gazebo::msgs::Plugin& GUI::plugin(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.GUI.plugin)
  return plugin_.Get(index);
}
inline ::gazebo::msgs::Plugin* GUI::mutable_plugin(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.GUI.plugin)
  return plugin_.Mutable(index);
}
inline ::gazebo::msgs::Plugin* GUI::add_plugin() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.GUI.plugin)
  return plugin_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >*
GUI::mutable_plugin() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.GUI.plugin)
  return &plugin_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >&
GUI::plugin() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.GUI.plugin)
  return plugin_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::GUI> GUIPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::GUI const> ConstGUIPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_gui_2eproto__INCLUDED
